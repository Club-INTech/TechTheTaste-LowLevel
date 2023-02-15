#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include "hardware/irq.h"

#include "./header/encoder.h"
#include "./header/PID.h"
#include "./header/motor.h"       
#include "./header/motion.h"

int consigne;                                   // How far we want to go,int cause ticks, it's the distance or angle we want

void init_all_enc_mot(){

    init_encoder(Signal_A_Right);
    init_encoder(Signal_A_Left);
    init_encoder(Signal_B_Right);
    init_encoder(Signal_B_Left);

    init_motor(Motor_R_For,&slice_R_For, &channel_R_For);
    init_motor(Motor_R_Rev,&slice_R_Rev, &channel_R_Rev);
    init_motor(Motor_L_For,&slice_L_For, &channel_L_For);
    init_motor(Motor_L_Rev,&slice_L_Rev, &channel_L_Rev);
}

void init_interrupt(){
    
    gpio_set_irq_enabled_with_callback(Signal_A_Right,GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, read_encoder);
    gpio_set_irq_enabled(Signal_A_Left, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(Signal_B_Right, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(Signal_B_Left, GPIO_IRQ_EDGE_RISE, true);

}

//Forward: positive distance ; Reverse: negative distance   
//HL job to convert the distance we want to go to a certain number of ticks and it give it to us as consigne 

//add distance between the two encoder, the delta in the command to the motors, see how to do that

void move_translate (int consigne){
    
    Actual_left = counter_Left;
    Actual_right = counter_Right;
    if (consigne <0){ 
        Actual_trans= abs((Actual_left+Actual_right)/2); 
        int Goal = -consigne;  
    }
    if (consigne>0){
        Actual_trans= (Actual_left+Actual_right)/2;  
        int Goal=consigne; 
    }
    float alpha_right= 1 - (Actual_right/consigne) ;
    float alpha_left= 1 - (Actual_left/consigne) ;

    dif_right= consigne - Actual_right ;
    dif_left = consigne - Actual_left ;

    printf("dif_r %i \n",dif_right);
    printf("dif_l %i \n",dif_left);

    Output_trans = PID(kP_trans, kI_trans,kD_trans, Actual_trans,Goal, &Sum_error_trans,&last_error_trans);
    Output_left = PID(kP_left,kI_left,kD_left,Actual_right, consigne, &Sum_error_left,&last_error_left);             //To synchronize the movement, we give to each PID the ticks count of the other encoder
    Output_right = PID(kP_right,kI_right, kD_right,Actual_left,consigne, &Sum_error_right, &last_error_right);

    command_left = Output_trans + (alpha_left * pwm_base) + ((1- alpha_left)*Output_left);
    command_right = Output_trans + (alpha_right * pwm_base) + ((1- alpha_right)*Output_right);
    

    //Command PWM right motor
    command_motors (slice_R_Rev,slice_R_For, channel_R_Rev, channel_R_For, command_right, dif_right); 

    //Command PWM left motor
    command_motors ( slice_L_Rev, slice_L_For,channel_L_Rev,channel_L_For, command_left, dif_left);
}


//angle converted to a certain number of ticks // HL job
//Clockwise : positive consigne ; Counter Clockwise : negative consigne (not sure at all, to check)
void move_rotate (int consigne){  

    int Goal_right = consigne;
    int Goal_left = -consigne;

    Actual_left = counter_Left;
    Actual_right = counter_Right;
    Actual_rot= Actual_right - Actual_left;                  

    float alpha_right= 1 - (Actual_right/Goal_right) ;
    float alpha_left= 1 - (Actual_left/Goal_left) ;

    dif_right= Goal_right - Actual_right ;
    dif_left= Goal_left - Actual_left;

    Output_rot = PID(kP_rot, kI_rot,kD_rot, Actual_rot, consigne, &Sum_error_rot, &last_error_rot); 
    Output_left = PID(kP_left,kI_left,kD_left,Actual_right,Goal_left, &Sum_error_left, &last_error_left);
    Output_right = PID(kP_right,kI_right, kD_right, Actual_left,Goal_right, &Sum_error_right, &last_error_right);

    command_left = Output_rot + (alpha_left * pwm_base) + ((1- alpha_left)*Output_left ) ;
    command_right = Output_rot + (alpha_right* pwm_base) + ((1- alpha_right)*Output_right);

    //Command PWM right motor
    command_motors (slice_R_Rev,slice_R_For, channel_R_Rev,channel_R_For,command_right, dif_right);

    //Command PWM left motor
    command_motors (slice_L_Rev,slice_L_For,channel_L_Rev,channel_L_For,command_left, dif_left);
    
}


