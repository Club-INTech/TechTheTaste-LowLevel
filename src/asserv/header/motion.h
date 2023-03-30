#pragma once

#define pwm_base 625                                                    //Fréquence de travail : 20 kHz, wrap à 6250, là PWM de base à 10% but provisoire

extern float kP_right;
extern float kD_right;
extern float kI_right;

extern float kP_left;
extern float  kD_left;
extern float kI_left;

extern float kP_trans;
extern float kD_trans;
extern float kI_trans;

extern float kP_rot;
extern float kD_rot;
extern float kI_rot;

extern int lidar;
extern int cancelmove;


extern int consigne;                                                    // How far we want to go,int cause ticks, it's the distance or angle we want
extern bool no_Lidar;
///extern absolute_time_t time_now;

static float Actual_left,Actual_right, Actual_trans , Actual_rot;       //Actual position of right and left encoder and for translation and rotation
static float Output_left, Output_right, Output_trans, Output_rot ;      //Results after PID, what we give to the motors        
static long int dif_right, dif_left;                                    //Let us know which way the motor is sup
static int dif;




void init_all_enc_mot();
void init_interrupt();


void move_translate (int consigne);
void move_rotate (int consigne);


//bool is_movement();


int determine_margin (int consigne);
bool translate (int consigne);
bool rotate (int consigne);


