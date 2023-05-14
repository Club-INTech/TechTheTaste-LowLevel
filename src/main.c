#include "hardware/irq.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include <pico/stdlib.h>
#include <com.h>
#include <stepper.h>
#include <motorpumpsvalve.h>
#include <pid.h>
#include <pidll.h>
#include <stdio.h> 

char ordermove[5]={0x10};
char orderrotate[5]={0x20};
char orderarm[5]={0x40};
int main(){
//	uartInit();
//	uartIrqSetup();	
	stdio_init_all();
	while (1) {
		receive();
		if(movestop){
			finish(ordermove);
			movestop=0;
		}
		if(rotatestop){
			finish(orderrotate);
			rotatestop=0;
		}
		if(armstop){
			finish(orderarm);
			armstop=0;
		}

	}
}


