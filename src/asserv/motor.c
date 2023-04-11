#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "./header/motor.h"

uint channel_L_For,channel_L_Rev, channel_R_For,channel_R_Rev;       //channels for the pwm signals to the motors
uint slice_L_For,slice_L_Rev,slice_R_For,slice_R_Rev;               // slices for the pwm signals to the motors

float command_left, command_right; 


void init_motor (uint gpio, uint *slice_num, uint *channel_num){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    (*slice_num) = pwm_gpio_to_slice_num(gpio);
    (*channel_num)=pwm_gpio_to_channel(gpio);

    pwm_set_wrap((*slice_num),6250);                                //we set the wrap at 6250 for a frequency of 20kHz
    pwm_set_chan_level((*slice_num),(*channel_num),0);
    pwm_set_enabled((*slice_num),true);
}


void command_motors (uint slice_Rev, uint slice_For, uint channel_Rev, uint channel_For, float command, long int sens){  

        if (command<0){
            command=-command;
        }
        if (command>6250){
            command=6250;
        }

        if (sens>0){//we want to move forward, turn the reverse off
            pwm_set_chan_level(slice_Rev,channel_Rev,0);
            pwm_set_chan_level(slice_For, channel_For, command); // command is the level 
        }
        else {//We want to move in reverse, turn the forward off
            pwm_set_chan_level(slice_For, channel_For,0);
            pwm_set_chan_level(slice_Rev, channel_Rev, command);
        }
}

