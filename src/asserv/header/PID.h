#pragma once



// #define timeChange 5*(pow (10,-5)) // PWM fréquence 20 kHz 5.0E-5


extern long Sum_error_right, last_error_right ;
extern long Sum_error_left, last_error_left;
extern long Sum_error_trans , last_error_trans;
extern long Sum_error_rot, last_error_rot;

long PID (float kP,float kI, float kD, long error, long *Sum_error, long *last_error);

