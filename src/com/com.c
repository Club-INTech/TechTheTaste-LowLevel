#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <com.h>
#include "hardware/regs/intctrl.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <stepper.h>
#include <motorpumpsvalve.h>
#include <pid.h>
#include <pidll.h>



/*------Object_definition------*/



float vitesse = 450;
static float *pid[16]={&kprot,&kdrot,&kirot,&kptrans,&kdtrans,&kitrans,&vitesse,NULL,NULL,NULL};
static struct repeating_timer tracktimer;
char buffer[5] ={0,0,0,0,0};
char syncBuffer[10]={0};
char order[5];
unsigned int id=0;
unsigned int comp=0;
unsigned short arg0=0;
unsigned short arg1=0;
static int interruptcounter = 0;
static char syncBuffertype[10] = { 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6 , 0xf7, 0xf8, 0xf9};
static char syncBuffer1[5] = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4};
static char syncBuffer2[5] = {0xf5, 0xf6 , 0xf7, 0xf8, 0xf9};
static int curve = 0;
static int bufferhead=0;

/*------UART_parameters------*/



#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1



/*------Order_array-----*/



void (*fonction[16]) (unsigned int comp, unsigned short arg0, unsigned short arg1) = { 
	*lidarStop,
	*move, 
	*rotatefunction,
	*cancelMove,
	*arm,
	NULL,
	*pumpsvalvemotors,	
	NULL,
	NULL,
	*setVar,
	*getVar,
	*track,
	NULL,
	NULL,
	NULL,
	*identification
	};



/*------UART_init------*/



void uartInit(){ // Setup uart parameters
	uart_init(UART_ID, 2400);
    gpio_set_function(UART_TX_PIN,GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID,true);
    }



void uartIrqSetup(){ //Setup receive irq on byts 
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, receiveParse);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
}


/*------Routine_uart------*/



void receiveParse() {//receive data and parsing 
	interruptcounter+=1;
	interruptcounter%=5;
	char ch=uart_getc(UART_ID);
	for(int k=0;k<4;k++){
		buffer[k]=buffer[k+1];
	}
	buffer[4]=ch;
	for(int k=0;k<9;k++){
		syncBuffer[k]=syncBuffer[k+1];
	}
	syncBuffer[9]=ch;
	if(tabEqual(syncBuffer,syncBuffertype,10)){	
		for(int k=0;k<5;k++){
			buffer[k]=0;
			interruptcounter=0;
		}
	}
	else if(tabEqual(buffer,syncBuffer1,5) | tabEqual(buffer,syncBuffer2,5)){
			return;
		}
	else if(interruptcounter==0){
		for(int k=0;k<5;k++){
			order[k]=buffer[k];
		}
		id   = getId(order[0]);
		comp = getComp(order[0]);
		arg1 = getArg1();
		arg0 = getArg0();
		fonction[id](comp,arg1,arg0);
		interruptcounter=0;
	}

}

void receive(){
	int ch = getchar_timeout_us(1);
	if(ch!=PICO_ERROR_TIMEOUT){
		buffer[bufferhead]=ch;
		bufferhead++;
		if(tabEqual(buffer,syncBuffer1,5)){
			bufferhead=0;
		}
		else if(tabEqual(buffer,syncBuffer2,5)){
			bufferhead=0;
		}
		else if(bufferhead==5){
			for(int k=0;k<5;k++){
				order[k]=buffer[k];
			}
			id   = getId(order[0]);
			comp = getComp(order[0]);
			arg1 = getArg1();
			arg0 = getArg0();
			fonction[id](comp,arg1,arg0);
			bufferhead=0;
		}
	}
}


void acknowledge(char order[5]){ //acknowledgement for recieved order 
	putchar_raw(order[0]>>4);
	for(int k=0; k<4;k++){
		putchar_raw(0x00);
		}
	}



void finish(char order[5]){ //acknowledgement for finished order 
	putchar_raw(0x10+(order[0]>>4));
	for(int k=0; k<4;k++){
		putchar_raw(0x00);
		}
	}



void sendVar(int data, int id, int comp){ //send variabes of pid 
	char byt1 = data >> 24;
	char byt2 = (data>>16)&0xFF;
	char byt3 = (data>>8)&0xFF;
	char byt4 = data&0xFF;
	putchar_raw((id<<4)+comp);
	putchar_raw(byt1);
	putchar_raw(byt2);
	putchar_raw(byt3);
	putchar_raw(byt4);
}



bool sendtrack(struct repeating_timer *t){ //send encoder data 
	putchar_raw(0x50);
	putchar_raw(((unsigned short) leftcounter>> 8 ) & 0xFF);
	putchar_raw(((unsigned short)leftcounter) & 0xFF);
	putchar_raw(((unsigned short) rightcounter >> 8) & 0xFF);
	putchar_raw(((unsigned short) rightcounter) & 0XFF);
	return true;
}



/*------Parsing_routines-------*/



int getId(char octet0) { //get the order id 
    char MASK = 0x0f;
    return (octet0 >> 4) & MASK;
	}

unsigned int getComp(unsigned int octet0){ //get the complement of the order id 
    return octet0 & 0x0f;
	}



unsigned short getArg1(){ //get the second part of the order 
    return (((unsigned short) order[1]) << 8) + ((unsigned short) order[2]);
} 



unsigned short getArg0(){ //get the frist part of the oder
	return (((unsigned short) order[3]) << 8) + ((unsigned short) order[4]);
}



float getFloat(unsigned short arg0, unsigned short arg1){ //transform int in float in order to store the value in the array 
    unsigned int tmp = (((unsigned int) arg0) << 16) + (unsigned int) arg1;
    float floatArg = *(float*) &tmp; //chelou
    return floatArg;
}



int getInt(float floatArg){ //transform float in int for communication
	int intarg=*(int*)&floatArg;
	return intarg;
}



int tabEqual(char tab1[], char tab2[],int lenght){ //check if tow arrays are the same 
	int nbs=0;
	for(int k=0;k<lenght;k++){
		nbs+=(tab1[k]==tab2[k])?1:0;
	}
	return nbs==lenght?1:0;
}



int ident(unsigned char type){ //ident the type of the pico
		putchar_raw(0x70+type);
	for(int k=0; k<4;k++){
		putchar_raw(0x00);
		}
	return 0;
}



/*------Order------*/



void lidarStop( unsigned int comp, unsigned short arg0, unsigned short arg1){ //use to stop the robot if lidar is trigered 
	acknowledge(order);
	lidarstop=!lidarstop;
	finish(order);
}


void move( unsigned int comp, unsigned short arg0, unsigned short arg1){ //move the robot in translation 
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN,GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN,0);
	acknowledge(order);
	//consigne= (short)arg1;
	movelow((short) arg1,(short) arg1);
}

void rotatefunction( unsigned int comp,unsigned short arg0, unsigned short arg1){ //rotate the robot on himself 
	acknowledge(order);
	//consigne= (short)arg1;
	movelow((short)arg1,-(short)arg1);
}



void cancelMove( unsigned int comp, unsigned short arg0, unsigned short arg1){ //cancel any move of the robot
	acknowledge(order);
	cancelmove=1;
	finish(order);
	}



void arm( unsigned int comp, unsigned short arg0, unsigned short arg1){ //move the robot arm 
	static char orderarm[5]={0x40,0x00,0x00,0x00,0x00};
	acknowledge(orderarm);
	int target[2]={(short) arg0, (short) arg1};
	armMove(target,(int)vitesse);
	}
	


void  pumpsvalvemotors( unsigned int comp, unsigned short arg0, unsigned short arg1){ //control on/off actuators such as pumps,valve,motor
	acknowledge(order);
	actionInit1A();
	uint8_t pumpmask=arg1&0xFF;
	uint8_t valvemask=arg0&0XFF;
	uint8_t motormask=(arg1>>8)&0XFF;
	switch (comp) {
		case 1:
			updatePumpValve(pumpmask,0,1);
		break;
		case 2:
			updatePumpValve(valvemask,1,1);
		break;
		case 3:
			updateMotor(motormask,0);
		break;
	}
	finish(order);
}



void setVar( unsigned int comp, unsigned short arg0, unsigned short arg1){ //set variable tab 
	acknowledge(order);
	*pid[comp]=getFloat(arg0,arg1);
	finish(order);
	
	}



void getVar( unsigned int comp, unsigned short arg0, unsigned short arg1){ //read the variable tab  
	acknowledge(order);
	sendVar(getInt(*pid[comp]),2,comp);
	finish(order);
	}



void track( unsigned int comp, unsigned short arg0, unsigned short arg1){ //encoder tracking for PID tuning 
	curve=!curve;
	if(curve){
		add_repeating_timer_ms(10,sendtrack,NULL,&tracktimer);
	
	}
	else{
		cancel_repeating_timer(&tracktimer);
	}
}



void identification(unsigned int comp, unsigned short arg0, unsigned short arg1){ //ident asserv=0  or action=1  raspberrypico
	acknowledge(order);
	ident(0);
	finish(order);
}





	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
