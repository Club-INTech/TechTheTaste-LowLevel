#include <stdint.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include <pidll.h>
#include <pidll.h>
#include <pid.h>
#include <com.h>
#define UART_ID uart0

/*------Order_syntax------*/
char ordercancel[5] = {0x30};
char ordermove[5]={0x10};
char orderrotate[5]={0x20};
 
/*------Wheels_target------*/
static int lefttarget;
static int righttarget;


/*------Timer_definition-----*/
struct repeating_timer samplingtimer; 
struct repeating_timer asservtimer;

/*------lidarstop cancelmove init------*/
int lidarstop=0;
int cancelmove=0;

/*------encoder_code------*/
int leftcounter=0;
int rightcounter=0;

void incrementIrq(uint gpio,uint32_t event){
	switch (gpio) {
		case 2:	
			if(gpio_get(3)==1){
				rightcounter++;
			}
			else{
				rightcounter--;
			}
		break;
		case 4:
			if(gpio_get(5)==1){
				leftcounter++;
			}
			else {
				leftcounter--;
			}
		break;
	}
}

int encoderIrqSetup(){
	gpio_init(3);
	gpio_set_dir(3,GPIO_IN);
	gpio_pull_down(3);
	gpio_set_function(2, GPIO_FUNC_SIO);
	gpio_set_dir(2, false);
	gpio_pull_down(2);
	gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &incrementIrq);
	gpio_init(5);
	gpio_set_dir(5,GPIO_IN);
	gpio_pull_down(5);
	gpio_set_function(4, GPIO_FUNC_SIO);
	gpio_set_dir(4, false);
	gpio_pull_down(4);
	gpio_set_irq_enabled_with_callback(4, GPIO_IRQ_EDGE_RISE, true, &incrementIrq);
	return 0;
}



/*------motor_control------*/

int initMotors(){
	int pin[4]={16,17,15,14};
	for(int k=0;k<4;k++){
	gpio_set_function(pin[k],GPIO_FUNC_PWM);
	int slice =pwm_gpio_to_slice_num(pin[k]);
	int channel = pwm_gpio_to_channel(pin[k]);
	pwm_set_wrap(slice,6250);                                
    pwm_set_chan_level(slice,channel,0);
    pwm_set_enabled(slice,true);
	}
	return 0;
}

int commandMotors (int pinforward,int pinbackward,double command){
	int slicerev = pwm_gpio_to_slice_num(pinbackward);
	int channelrev = pwm_gpio_to_channel(pinbackward);
	int slicefor = pwm_gpio_to_slice_num(pinforward);
	int channelfor = pwm_gpio_to_channel(pinforward);
	if(lidarstop|cancelmove){
		pwm_set_chan_level(slicerev,channelrev,0);
		pwm_set_chan_level(slicefor,channelfor,0);
	}
	else{
		if (command>=0){
			pwm_set_chan_level(slicerev,channelrev,0);
			pwm_set_chan_level(slicefor, channelfor, command); // command is the level 
		}
		else{
			pwm_set_chan_level(slicefor, channelfor,0);
			pwm_set_chan_level(slicerev, channelrev, -command);
		}	
	}
	return 0;
}

/*------sampling------*/

bool timerPID(struct repeating_timer *t){
	updateArchi(&archi,leftcounter,rightcounter);
	return true;
}

/*------asserv_startup------*/

bool timerAsserv(struct repeating_timer *t){
	command(&archi);
	if(cancelmove){
		command(&archi);
		removeTimer();
		finish(ordercancel);
		cancelmove=0;
		removeTimer();
	}
	return true;
}


/*-----timers------*/

int initTimers(){
	add_repeating_timer_ms(5,timerPID,NULL,&samplingtimer);
	sleep_ms(1000);
	add_repeating_timer_ms(10,timerAsserv,NULL,&asservtimer);
	return 0;
}

int removeTimer(){
	cancel_repeating_timer(&samplingtimer);
	cancel_repeating_timer(&asservtimer);
}


/*------pid_control------*/

int command(PIDArchi* PIDArchi){
	double	outputleft=getArchiLeftOutput(PIDArchi);
	double outputright=getArchiRightOutput(PIDArchi);
	commandMotors(16,17,outputright);
	commandMotors(14,15,outputleft);
}

/*------order_implementation------*/

int movelow(int consigneleft, int consigneright){
	removeTimer();
	encoderIrqSetup();
	initMotors();
	initPID(&distance,&kptrans,&kitrans,&kdtrans);
	initPID(&direction,&kprot,&kirot,&kdrot);
	initArchi(&archi, &distance, &direction);
	resetArchi(&archi,consigneleft,consigneright);
	initTimers();
}
	

























