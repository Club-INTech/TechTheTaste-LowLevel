#include <stdint.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <pidll.h>
#include <pidll.h>
#include <pid.h>

/*------encoder_code------*/
int leftcounter=0;
int rightcounter=0;
int cancelmove=0;
int lidarstop=0;


#define Signal_A_Right 19
#define Signal_B_Right 18

#define Signal_A_Left 17
#define Signal_B_Left 16

void incrementIrq(uint gpio,uint32_t events){
	switch (gpio) {
		case 19:	
			if(gpio_get(18)==1){
				rightcounter++;
			}
			else{
				rightcounter--;
			}
		break;
		case 17:
			if(gpio_get(16)==1){
				leftcounter++;
			}
			else {
				leftcounter--;
			}
		break;
	}
}

int encoderIrqSetup(){
	gpio_init(18);
	gpio_set_dir(18,GPIO_IN);
	gpio_pull_down(18);
	gpio_set_function(19, GPIO_FUNC_SIO);
	gpio_set_dir(19, false);
	gpio_pull_down(19);
	gpio_set_irq_enabled_with_callback(19, GPIO_IRQ_EDGE_RISE, true, &incrementIrq);
	gpio_init(16);
	gpio_set_dir(16,GPIO_IN);
	gpio_pull_down(16);
	gpio_set_function(17, GPIO_FUNC_SIO);
	gpio_set_dir(17, false);
	gpio_pull_down(17);
	gpio_set_irq_enabled_with_callback(17, GPIO_IRQ_EDGE_RISE, true, &incrementIrq);
	return 0;
}



/*------motor_control------*/

#define Motor_R_For 26  // right motor, forward
#define Motor_R_Rev 27 // right motor, reverse

#define Motor_L_For 20 // left motor, forward
#define Motor_L_Rev 21 // left motor, reverse


int initMotors(){
	int pin[4]={26,27,20,21};
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
	
	if (command>=0){
		pwm_set_chan_level(slicerev,channelrev,0);
        pwm_set_chan_level(slicefor, channelfor, command); // command is the level 
    }
	else{
		pwm_set_chan_level(slicefor, channelfor,0);
        pwm_set_chan_level(slicerev, channelrev, -command);
    }
	return 0;
}

/*------sampling------*/

struct repeating_timer pidtimer; 

bool timerPID(struct repeating_timer *t){
	updateArchi(&archi,leftcounter,rightcounter);
	return true;
}

int initTimer(){
	add_repeating_timer_ms(5,timerPID,NULL,&pidtimer);
	return 0;
}

int removeTimer(){
	cancel_repeating_timer(&pidtimer);
}


/*------pid_control------*/

int movelow(PIDArchi* PIDArchi){

	double outputleft=getArchiLeftOutput(PIDArchi);
	double outputright=getArchiRightOutput(PIDArchi);
	if(lidarstop|cancelmove){
		commandMotors(26,27,0);
		commandMotors(20,21,0);
		return 1;
	}
	else{
		commandMotors(26,27,outputright);
		commandMotors(20,21,outputleft);
		return 0;
	}
}





























