#pragma once 
typedef struct PIDController {
    float *Kp, *Ki, *Kd;
    double error, derivative, integral;
} PIDController;



typedef struct {
    PIDController *left, *right, *central;
    double command_left, command_right;
	int cnt;
	int limit; 

} PIDArchi;

void initPID(PIDController*, float*, float*, float*);
void resetPID(PIDController*);
void updatePID(PIDController*, float);
double getPIDOutput(PIDController*);

void initArchi(PIDArchi*, PIDController*, PIDController*, PIDController*);
void resetArchi(PIDArchi*, double, double);
void updateArchi(PIDArchi*, double, double);

double getArchiLeftOutput(PIDArchi*);
double getArchiRightOutput(PIDArchi*);

extern PIDController center;
extern PIDController left;
extern PIDController right;
extern PIDArchi archi; 

extern float kP_right;
extern float kD_right; 
extern float kI_right; 
extern float kP_left;
extern float kD_left;
extern float kI_left;
extern float kP_center;
extern float kD_center;
extern float kI_center;
