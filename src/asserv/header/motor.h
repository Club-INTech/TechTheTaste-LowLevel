#pragma once


#define Motor_R_For 26  // right motor, forward
#define Motor_R_Rev 27 // right motor, reverse

#define Motor_L_For 20 // left motor, forward
#define Motor_L_Rev 21 // left motor, reverse


extern uint channel_L_For,channel_L_Rev, channel_R_For,channel_R_Rev; //channels for the pwm signals to the motors
extern uint slice_L_For,slice_L_Rev,slice_R_For,slice_R_Rev; // slices for the pwm signals to the motors

extern float command_left, command_right; // the commands we give the motors


void init_motor (uint gpio, uint *slice_num, uint *channel_num);

void command_motors (uint slice_Rev, uint slice_For, uint channel_Rev, uint channel_For, float command, long int sens);

