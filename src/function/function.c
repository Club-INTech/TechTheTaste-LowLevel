#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <function.h>
#include <com.h>
#include <pid.h>
#include <pidll.h>
#include <stepper.h>
#include <motorpumpsvalve.h>


float vitesse = 450;
static char ordermotors[5]={0x70,0x00,0x00,0x00,0x00}; 
static uint nbmotors=0;
static short positionmotors[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint nbordermotors=0;
static float *pid[16]={&kP_right,&kD_right,&kI_right,&kP_left,&kD_left,&kI_left,&kP_center,&kD_center,&kI_center,&vitesse,NULL,NULL,NULL,NULL,NULL,NULL};
static char ordercancelmove[5]={0x30,0x00,0x00,0x00,0x00};
static char orderlidarstop[5]={0x00,0x00,0x00,0x00,0x00};
static int curve=0;
struct repeating_timer timer;
	



void lidarStop( unsigned int comp, unsigned short arg0, unsigned short arg1){
	
}

void move( unsigned int comp, unsigned short arg0, unsigned short arg1){
	encoderIrqSetup();
	initMotors();
	initPID(&center,&kP_center,&kI_center,&kD_center);
	initPID(&left,&kP_left,&kI_left,&kD_left);
	initPID(&right,&kP_right,&kI_right,&kD_right);
	initArchi(&archi,&left,&right,&center);
	initTimer(); 
	resetArchi(&archi,(short)arg1, (short) arg1);
	acknowledge(order);
	while(1){
		if(movelow(&archi)){
			finish(ordercancelmove);
			cancelmove=0;
			removeTimer();
			cancel_repeating_timer(&timer);
			return;
		}
	}
	removeTimer();
	finish(order);
}

void rotatefunction( unsigned int comp, unsigned short arg0, unsigned short arg1){
	encoderIrqSetup();
	initMotors();
	initPID(&center,&kP_center,&kD_center,&kI_center);
	initPID(&left,&kP_left,&kD_left,&kI_left);
	initPID(&right,&kP_right,&kD_right,&kI_right);
	initArchi(&archi,&left,&right,&center);
	initTimer(); 
	resetArchi(&archi,-(short)arg1, (short) arg1);
	acknowledge(order);
	while(1){
		movelow(&archi);
		if(movelow(&archi)){
			finish(ordercancelmove);
			cancelmove=0;
			removeTimer();
			return;
		}
	}
	removeTimer();
	finish(order);
}


void cancelMove( unsigned int comp, unsigned short arg0, unsigned short arg1){
	
	
	}


void arm( unsigned int comp, unsigned short arg0, unsigned short arg1){
	static char orderarm[5]={0x40,0x00,0x00,0x00,0x00};
	acknowledge(orderarm);
	int target[2]={(short) arg0, (short) arg1};
	armMove(target,(int)vitesse);
	}
	


void motorTime( unsigned int comp, unsigned short arg0, unsigned short arg1){

}


void  pumpsvalvemotors( unsigned int comp, unsigned short arg0, unsigned short arg1){
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


void  motors( unsigned int comp, unsigned short arg0, unsigned short arg1){
	acknowledge(order);	
	int target[2]={arg0,arg1};


	/*	acknowledge(order);
	nbmotors=comp;*/
	
}


void motorsArgs( unsigned int comp, unsigned short arg0, unsigned short arg1){
	if(nbmotors>nbordermotors){
		positionmotors[nbordermotors]=arg1;
		nbordermotors++;
		acknowledge(order);
	}
	if(nbmotors==nbordermotors){
		nbordermotors=0;
		finish(ordermotors);
	}
}


void setVar( unsigned int comp, unsigned short arg0, unsigned short arg1){
	acknowledge(order);
	*pid[comp]=getFloat(arg0,arg1);
	finish(order);
	
	}
void getVar( unsigned int comp, unsigned short arg0, unsigned short arg1){
	acknowledge(order);
	sendVar(getInt(*pid[comp]),2,comp);
	finish(order);
	}
void track( unsigned int comp, unsigned short arg0, unsigned short arg1){
	acknowledge(order);
	curve=!curve;
	if(curve){
		add_repeating_timer_ms(100,sendtrack,NULL,&timer);
	
	}
	else{
		cancel_repeating_timer(&timer);
	}
	finish(order);
}

void identification(unsigned int comp, unsigned short arg0, unsigned short arg1){
	acknowledge(order);
	ident(0);
	finish(order);
}


void syncro(unsigned int comp, unsigned short arg0, unsigned short arg1){
	int a=2; 
	a++;
}


void mainprocess(){



	
	void (*fonction[16]) (unsigned int comp, unsigned short arg0, unsigned short arg1) = {
		NULL,
		*move, 
		*rotatefunction,
		NULL,
		*arm,
		*motorTime,
		*pumpsvalvemotors,	
		*motors,
		*motorsArgs,
		*setVar,
		*getVar,
		*track,
		NULL,
		NULL,
		NULL,
		NULL
		};
	
	const uint LED_PIN = PICO_DEFAULT_LED_PIN;
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	
	BufferInit(&buffer);
	uartInit();
	uartIrqSetup();
    while (1) {
			int flag=!ReadNewOrder(order,&buffer);
			id = getID(order[0]);
			comp = getCOMP(order[0]);
			arg0 = (((unsigned short) order[1]) << 8) + ((unsigned short) order[2]);
			arg1 = (((unsigned short) order[3]) << 8) + ((unsigned short) order[4]);
			if(id==10){	gpio_put(LED_PIN,1);}
			if(flag){
				fonction[id](comp,arg0,arg1);
				orderExecuted++;
			
		}
	}
}








