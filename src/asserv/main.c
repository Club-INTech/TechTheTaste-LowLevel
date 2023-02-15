#include <stdio.h>

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
    
    
    //demander type de mouvement : translation ou rotation
    //demander consgine : combien de ticks 

    // fonction à lancer en prenant en compte le fait qu'il est peut être déjà occupé ?
    // Comment savoir si mouvement toujours en cours ? Voir si on a pas reçu de signaux des codeuses depuis quelques instants (genre 10-100 ms ?)


    consigne= 1000; 

    while(1){

        move_translate(consigne);


    }

 
    return 0;
}

