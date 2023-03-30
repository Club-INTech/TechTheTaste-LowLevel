#pragma once


#define Signal_A_Right 19
#define Signal_B_Right 18

#define Signal_A_Left 17
#define Signal_B_Left 16

extern long int counter_Right;
extern long int counter_Left; 

//extern absolute_time_t time_int;


void init_encoder (uint gpio);
void read_encoder (uint gpio , uint32_t events);
void init_counters_encoders(long int *counter_Right, long int *counter_Left);