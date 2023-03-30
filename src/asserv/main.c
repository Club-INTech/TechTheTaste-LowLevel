#include <stdio.h>
#include <time.h>
#include <stdbool.h>

#include "./header/encoder.h"
#include "./header/motor.h"
#include "./header/PID.h"
#include "./header/motion.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "hardware/irq.h"



int main (){

    stdio_init_all();                                           //Allows us to read on the minicom

    init_all_enc_mot();                                         //Initialize all gpios motors and encoders

    init_interrupt(); 	                                        //Sets up the interruptions for the encoders so that they start counting 
    


    init_counters_encoders(&counter_Left, &counter_Right);      //Reinitializes the values of the counters to 0

    consigne  = 1000; 

   while (rotate(consigne)){
        move_rotate(consigne);
    }

 
    return 0;
}

