

#pragma once 

#include "pico/time.h"
#include <stdint.h>
#include <pid.h>

extern int leftcounter;
extern int rightcounter;
extern int lidarstop;
extern int cancelmove;

typedef struct{
	int pina;
	int pinb;
	int counter; 
}encoder; 

typedef struct{
	int pin;
}motor;

int encoderIrqSetup();
void incrementIrq(uint gpio,uint32_t event);
int initMotors();
int commandMotors (int pinforward,int pinbackward,double command);
int initTimers();
int removeTimer();
int command(PIDArchi* PIDArchi);
int movelow(int consigneleft,int consigneright);
bool timerAsserv(struct repeating_timer *t);
bool timerPID(struct repeating_timer *t);
extern struct repeating_timer samplingtimer; 
extern struct repeating_timer asservtimer;

extern int outputleft;
extern int outputright;
