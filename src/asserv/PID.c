#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "./header/PID.h"

long Sum_error_right=0, last_error_right=0 ;
long Sum_error_left=0, last_error_left=0;
long Sum_error_trans=0 , last_error_trans=0;
long Sum_error_rot=0, last_error_rot=0;


long PID (float kP,float kI, float kD, long error, long *Sum_error, long *last_error) {

	 //We can remove the multiplication by dt because it is always the same time (time to send the PWM signal), it will just be reflected in the kI and kD

	(*Sum_error) += error;  							
	long der_error = (error - (*last_error));
	return (kP * error) + (kI * (*Sum_error)) + (kD * der_error);

}


