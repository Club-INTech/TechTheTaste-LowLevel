#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"

#include "./header/encoder.h"


long int counter_Right;
long int counter_Left; 

//absolute_time_t time_int;

void init_encoder (uint gpio){

    gpio_init(gpio);
    gpio_set_dir(gpio,GPIO_IN);
    gpio_pull_down(gpio);

} 

void increase (long int *counter){
    (*counter) ++ ;
}
void decrease (long int *counter){
    (*counter) -- ;
}

void init_counters_encoders(long int *counter_Right, long int *counter_Left){
    (*counter_Right)=0;
    (*counter_Left)=0; 
}


// Same change = Counter clock wise
//Not same change = clock wise s


void read_encoder (uint gpio , uint32_t events){
    static uint32_t event_R; // initialized once first time we call function
    static uint32_t  event_L; //samesies

    if (gpio == Signal_A_Right){
        event_R=events;
    }

    if (gpio==Signal_B_Right){
        if (events == event_R ){
            increase(&counter_Right);
        }
        else {
            decrease(&counter_Right);
        }
    }

    if (gpio == Signal_A_Left){
       event_L=events;
    }

    if (gpio==Signal_B_Left){
        if (events == event_L ){
            increase(&counter_Left);
        }
        else {
            decrease(&counter_Left);
        }
    }

    //(*time_int)=get_absolute_time();

}

