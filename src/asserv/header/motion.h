#pragma once

#define pwm_base  500                                                  //Fréquence de travail : 20 kHz, wrap à 6250, là PWM de base à 50% but provisoire


extern int consigne;                                                    // How far we want to go,int cause ticks, it's the distance or angle we want

static float Actual_left,Actual_right, Actual_trans , Actual_rot;       //Actual position of right and left encoder and for translation and rotation
static float Output_left, Output_right, Output_trans, Output_rot ;      //Results after PID, what we give to the motors        
static long int dif_right, dif_left;                                    //Let us know which way the motor is sup
static int Goal;



void init_all_enc_mot();
void init_interrupt();


void move_translate (int consigne);
void move_rotate (int consigne);
