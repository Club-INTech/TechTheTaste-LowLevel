#pragma once




#define timeChange exp (pow (10,-9)) // Temps d'un cycle PWM


extern float Sum_error_right, last_error_right ;
extern float Sum_error_left, last_error_left;
extern float Sum_error_trans , last_error_trans;
extern float Sum_error_rot, last_error_rot;


double PID (float kP,float kI, float kD, float Input, float Setpoint, float *Sum_error, float *last_error);

