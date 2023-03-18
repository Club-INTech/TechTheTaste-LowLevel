#include "hardware/gpio.h"
#include "hardware/regs/intctrl.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/time.h"
#include "pico/types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <com.h>
#include<stepper.h>


static int count[8] = {0,0,0,0,0,0,0,0};
static stepper motors[8];
static endstop endstops[8];
static uint motormask = 0;
static int homed = 0;
static char ordermotors[5]={0x70,0x00,0x00,0x00,0x00}; 

double clockDivider(int speed){
	return ceil(125000000.0/(speed*4096))/16;
}

int	wrapCalculator(int speed){
	double clkfreq = 125000000*16/ceil(125000000.0/(speed*4096));
	return floor(clkfreq/speed);
}

void endstoprise(uint gpio ,uint32_t event_mask){
	int i=0;
	int k=0;
	while(endstops[k].pin!=gpio){
		k++;
	}
	while(endstops[k].axis!=motors[i].axis){
		i++;
	}
	pwm_set_enabled(motors[i].pwmSlice,false);
}


int homming(stepper *stepper, endstop *endstop,int dir){
	gpio_put(stepper->dirPin,dir);
	stepper->dir=dir;
	gpio_set_irq_enabled_with_callback(endstop->pin,GPIO_IRQ_EDGE_RISE,true,endstoprise);
	pwm_set_clkdiv(stepper->pwmSlice,clockDivider(stepper->speed));
	int wrap =wrapCalculator(stepper->speed);
	pwm_set_wrap(stepper->pwmSlice,wrap);
	pwm_set_chan_level(stepper->pwmSlice,stepper->pwmChan,wrap/2);
	pwm_set_enabled(stepper->pwmSlice,true);

}

int endstopInit(endstop *endstop){
	gpio_set_function(endstop->pin,GPIO_FUNC_SIO);
	gpio_set_dir(endstop->pin, false);
	gpio_pull_down(endstop->pin);
}



int maskMotor(stepper *motor){
	motormask=motormask|(1<<motor->pwmSlice);
	return 0;
}

void stepperCountDown(){
	int tozero=0;
	uint32_t mask=pwm_get_irq_status_mask();
	for(int k=0; k<8; k++){
		if(mask & 1){
			pwm_clear_irq(k);
			if(count[k]>0){
				count[k]--;
			}
			else{
				tozero++;
				pwm_set_irq_enabled(k,false);	
				pwm_set_enabled(k,false);
			}
		}
		mask=mask>>1;
	}
	if(tozero==8){
		finish(ordermotors);
	}
}

int irqPwmSetup(){
	if(!irq_has_shared_handler(PWM_IRQ_WRAP)){
		irq_add_shared_handler(PWM_IRQ_WRAP,stepperCountDown, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
		irq_set_enabled(PWM_IRQ_WRAP,true);
	}
}

int irqStepperSetup(stepper *stepper){
	pwm_clear_irq(stepper->pwmSlice);
	pwm_set_irq_enabled(stepper->pwmSlice,true);
	return 0;
}

int stepperInit(stepper *stepper,int id,int dirPin, int stepPin, int speed,int axis){
	stepper->id=id; 
	stepper->dirPin=dirPin;
	stepper->stepPin=stepPin;
	stepper->speed=speed;
	stepper->position=0;
	stepper->dir=0;
	gpio_set_function(stepper->stepPin, GPIO_FUNC_PWM);
	gpio_init(dirPin);
	gpio_set_dir(dirPin, GPIO_OUT);
	stepper->pwmSlice=pwm_gpio_to_slice_num(stepPin);
	stepper->pwmChan=pwm_gpio_to_channel(stepPin);
	motors[id-1]=*stepper;
	maskMotor(stepper);
	stepper->axis=axis;
	return 0;
}

int home(stepper stepper[],endstop endstop[], int dir [],int orderlenght){


}

int motorValueStepper(stepper motors[],int id[], int dir[], int target[],int orderlenght){
	for(int k=0;k<orderlenght;k++){
		for( int i=0;i<8;i++){
			if(id[k]==motors[i].id){
				motors[i].dir=dir[k];
				gpio_put(motors[i].dirPin,dir[k]);
				irqPwmSetup();
				irqStepperSetup(&motors[i]);
				count[motors[i].pwmSlice]=target[k];
				pwm_set_clkdiv(motors[i].pwmSlice,clockDivider(motors[i].speed));
				int wrap =wrapCalculator(motors[i].speed);
				pwm_set_wrap(motors[i].pwmSlice,wrap);
				pwm_set_chan_level(motors[i].pwmSlice,motors[i].pwmChan,wrap/2);
			}
		}
	}

	pwm_set_mask_enabled(motormask);
	return 0;
}

int armMove(int target[]){
	int utarget[2]={0,0};
	int dir[2]={0,0};
	int id[2]={1,2};
	for(int k=0;k<2;k++){
		if(target[k]<0){
			dir[k]=0;
		}
		else{
			dir[k]=1;
		}
		utarget[k]=abs(target[k]);
	}
	stepper motor1;
	stepper motor2;
	stepperInit(&motor1,1,7,6,450,1);
	stepperInit(&motor2,2,9,8,450,2);
	motorValueStepper(motors,id,dir,utarget,2);
}


	/*stepper motors1;
	stepper motors2;
	int id[2] = {1,2};
	int dir[2] = {0,0};
	int dir1[2] = {1,1};
	int target[2] = {200,200};
	stepperInit(&motors1,1,7,6,450);
	stepperInit(&motors2,2,9,8,450);
	for(int k=0; k<100; k++){
 
		motorValueStepper(motors,id,dir,target);
		sleep_ms(1000);
		motorValueStepper(motors,id,dir1,target);
		sleep_ms(1000);

*/

