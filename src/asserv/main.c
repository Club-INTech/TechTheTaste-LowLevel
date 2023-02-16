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


    stdio_init_all();       //Permet de lire sur le minicom

    init_all_enc_mot();     //Initialise tous les gpio  moteurs et encodeurs

    init_interrupt(); 	    //Set up the interruptions for the encoders so that they start counting 
    
    sleep_ms(1000);
    
    //demander type de mouvement : translation ou rotation
    //demander consgine : combien de ticks 


    consigne  = 1000; 


   while (rotate(consigne)){
        move_rotate(consigne);
    }

 
    return 0;
}

