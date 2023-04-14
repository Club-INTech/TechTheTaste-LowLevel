#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <com.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "hardware/regs/intctrl.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include <pid.h>
#include <pidll.h>


CircularBuffer buffer;
char syncBuffer[10] ={0,0,0,0,0,0,0,0,0,0};
char order[5];
int orderExecuted=0;
unsigned int id=0;
unsigned int comp=0;
unsigned short arg0=0;
unsigned short arg1=0;
static char ordercancelmove[5]={0x30,0x00,0x00,0x00,0x00};
static char orderlidarstop[5]={0x00,0x00,0x00,0x00,0x00};
static int first = 1;
static char syncBuffertype[10] = { 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6 , 0xf7, 0xf8, 0xf9};
int sync = 0;


/*definition of uart communication parameter */

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1


/*UART init*/

void uartInit(){ // Setup uart parameters
	uart_init(UART_ID, 2400);
    gpio_set_function(UART_TX_PIN,GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true);
    }


void uartIrqSetup(){ //Setup receive irq on byts 
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, receive);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
}


/* Routine used for communication wich deal with uart*/

void receive() {//receive data and write the buffer
	volatile char ch=uart_getc(UART_ID);
	for(int k=0;k<9;k++){
		syncBuffer[k]=syncBuffer[k+1];

	}
	syncBuffer[9]=ch;
	sync=tabEqual(syncBuffer,syncBuffertype);
	WriteBuffer(ch,&buffer);
	if(((buffer.Head)%5-1)==0){
		int idirq=getID(ch);
		if(idirq==0){
				acknowledge(orderlidarstop);
				lidarstop=!lidarstop;
				finish(orderlidarstop);
		}
		else if(idirq==3){
				acknowledge(ordercancelmove);
				cancelmove=1;
	
		}
	}
}

void acknowledge(char order[5]){ //acknowledgement for recieved order 
	uart_putc_raw(UART_ID,order[0]>>4);
	for(int k=0; k<4;k++){
		uart_putc(UART_ID, 0x00);
		}
	}

void finish(char order[5]){ //acknowledgement for finished order 
	uart_putc_raw(UART_ID,0x10+(order[0]>>4));
	for(int k=0; k<4;k++){
		uart_putc(UART_ID, 0x00);
		}
	}

void sendVar(int data, int id, int comp){
	char byt1 = data >> 24;
	char byt2 = (data>>16)&0xFF;
	char byt3 = (data>>8)&0xFF;
	char byt4 = data&0xFF;
	uart_putc(UART_ID,(id<<4)+comp);
	uart_putc(UART_ID,byt1);
	uart_putc(UART_ID,byt2);
	uart_putc(UART_ID,byt3);
	uart_putc(UART_ID,byt4);
}

bool sendtrack(struct repeating_timer *t){
	uart_putc(UART_ID,0x50);
	uart_putc(UART_ID,((unsigned short) leftcounter >> 8 ) & 0xFF);
	uart_putc(UART_ID,((unsigned short)	leftcounter) & 0xFF);
	uart_putc(UART_ID,((unsigned short) rightcounter >> 8) & 0xFF);
	uart_putc(UART_ID,((unsigned short)	rightcounter) & 0XFF);
	return true;
}
	
/*Buffer management routines*/

void BufferInit(CircularBuffer *Buffer){ //init buffer parameter	
	Buffer->Head=0;
	Buffer->Tail=0;
	Buffer->BufferSize=1000;
	Buffer->BufferFullFlag=0;
	Buffer->BufferEmptyFlag=1;
	Buffer->BufferOrderNumber=0;
	for(int k=0;k<Buffer->BufferSize;k++){
		Buffer->Buffer[k]=0;
	}
};

int WriteBuffer(char data, CircularBuffer *Buffer){ //write buffer and update related flags
	if(Buffer->BufferFullFlag){ 
		return 1;
		}
	else{
		Buffer->Buffer[Buffer->Head] = data;
		Buffer->Head = (Buffer->Head+1)%(Buffer->BufferSize);
		Buffer->BufferEmptyFlag=0;
		if(Buffer->Head == Buffer->Tail){
			Buffer->BufferFullFlag=1;
			}
		if(Buffer->Head%5==0){
			Buffer->BufferOrderNumber++;
			}
		return 0;
		}
	}


int IsBufferEmpty(CircularBuffer *Buffer){ //check if the buffer is empty and update related flags
	int temp=0;
	if(Buffer->Head == Buffer->Tail &&  Buffer->BufferFullFlag!=1){
		temp=1;
		}
	Buffer->BufferEmptyFlag=temp;
	return temp;
	}

int ReadBuffer(char *data,  CircularBuffer *Buffer){ //read the buffer and update related flags
	if(IsBufferEmpty(Buffer)){
		return 1;
		}
	else{
		*data=Buffer->Buffer[Buffer->Tail];
		Buffer->Tail= (Buffer->Tail+1)%(Buffer->BufferSize);
		Buffer->BufferFullFlag=0;
		return 0;
		}
	}


int ReadNewOrder(char order[5],CircularBuffer *Buffer ){ //extract order from the buffer 
	int result=0;
	for(int k=0; k<5;k++){
		char temp;
		result = ReadBuffer(&temp,Buffer);
		order[k]=temp;
		}
	return result;
	}

/* Parsing routines */

unsigned long extraire_bit(unsigned long data, int n) {
	unsigned long MASK = 0x1;
    unsigned long bit = (data >> n) & MASK;
	return bit;
	}

unsigned int concatene16(unsigned int octet1, unsigned int octet2){
    unsigned int result = (uint16_t)(octet1 << 8)|octet2;
    return result;
	}

unsigned int concatene32(unsigned int double_octet1, unsigned int double_octet2){
    unsigned int result = (uint32_t)(double_octet1 << 8)|double_octet2;
    return result;
	}


int getID(char octet0) {
    char MASK = 0x0f;
    return (octet0 >> 4) & MASK;
	}

unsigned int getCOMP(unsigned int octet0){
    return octet0 & 0x0f;
	}

unsigned int getARG(unsigned int double_octet){
    return double_octet >> 0;
	}

float getFloat(unsigned short arg0, unsigned short arg1){
	
    unsigned int tmp = (((unsigned int) arg0) << 16) + (unsigned int) arg1;
    float floatArg = *(float*) &tmp; //chelou

    return floatArg;
}

int getInt(float floatArg){
	int intarg=*(int*)&floatArg;
	return intarg;
}


int tabEqual(char tab1[10], char tab2[10]){	
	int nbs=0;
	for(int k=0;k<10;k++){
		nbs+=(tab1[k]==tab2[k])?1:0;
	}
	return nbs==10?1:0;
}

int ident(unsigned char type){
	uart_putc_raw(UART_ID,0x70+type);
	for(int k=0; k<4;k++){
		uart_putc(UART_ID, 0x00);
		}
	return 0;
}


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
