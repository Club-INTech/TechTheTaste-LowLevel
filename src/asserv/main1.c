#include <stdint.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/structs/interp.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include <pidll.h>
#include <pid.h>

#define abs(x) ((x) < 0) ? -(x) : (x)

int _left, _right;


void lidar(uint gpio, uint32_t events){
	while(gpio_get(gpio)==1){
	}
}

void _move(int m){
	uint32_t date = to_ms_since_boot(get_absolute_time());
	_left += m;
	_right += m;
	resetArchi(&archi, _left, _right);
	while ((abs(rightcounter - _right) < 50 || abs(leftcounter - _left) < 50) &&  (to_ms_since_boot(get_absolute_time())-date < 1500)) {
		command(&archi);
		printf("%d,%d",date,to_ms_since_boot(get_absolute_time()));
	}
	commandMotors(16,17,0);
	commandMotors(14,15,0);
}

void _rotate(int m) {
	uint32_t date = to_ms_since_boot(get_absolute_time());
	_left -= m;
	_right += m;
	resetArchi(&archi, _left, _right);
	while ((abs(rightcounter - _right) < 100 || abs(leftcounter - _left) < 100) && (to_ms_since_boot(get_absolute_time())-date < 1500)) {
		command(&archi);
		printf("%d,%d \n",date,to_ms_since_boot(get_absolute_time()));

	}
	commandMotors(16,17,0);
	commandMotors(14,15,0);
}





float pid[16] ={20,38,5,5,15,3};
int main(){
	gpio_init(1);
	gpio_set_dir(1,GPIO_IN);
	gpio_set_irq_enabled_with_callback(1,GPIO_IRQ_EDGE_RISE,true,lidar);
	gpio_init(0);
	gpio_set_dir(0,GPIO_IN);
	gpio_pull_up(0);
	while(1){
		int k=0;
		for(int i=0;i<10;i++) k+=(gpio_get(0)==0)?1:0;
		if(k==10){
			break;
		}
	}
	stdio_init_all();
	_left = 0;
	_right = 0;
	stdio_init_all();
	encoderIrqSetup();
	initMotors();
	initPID(&distance,&pid[3],&pid[4],&pid[5]);
	initPID(&direction,&pid[0],&pid[1],&pid[2]);	
	initArchi(&archi, &distance, &direction);
	initTimers();
	for (int i=0; i < 3; i++) {
		_move(1000);
	}
	_rotate(-200);
	_move(1000);
	_rotate(-200);
	for (int i=0; i < 2; i++) {
		_move(1000);
	}
	for (int i=0; i < 3; i++) {
		_rotate(220);
	}
	
	_move(1000);
	_move(1000);
	_move(1000);
}
	

