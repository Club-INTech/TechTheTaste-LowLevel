#pragma once 
typedef struct PIDController {
    float *Kp, *Ki, *Kd;
    double error, derivative, integral;
	double memory[100];
} PIDController;

typedef struct {
    PIDController *direction, *distance;
    double command_distance, command_direction;
	int cnt, limit;
} PIDArchi;

void initPID(PIDController*, float*, float*, float*);
void resetPID(PIDController*);
void updatePID(PIDController*, float);
double getPIDOutput(PIDController*);

void initArchi(PIDArchi*, PIDController*, PIDController*);
void resetArchi(PIDArchi*, double, double);
void updateArchi(PIDArchi*, double, double);

double getArchiLeftOutput(PIDArchi*);
double getArchiRightOutput(PIDArchi*);

extern PIDController direction;
extern PIDController distance;
extern PIDArchi archi;
extern float kprot;
extern float kdrot;
extern float kirot;
extern float kptrans;
extern float kdtrans;
extern float kitrans;



