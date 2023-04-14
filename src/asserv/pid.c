#include "pid.h"
#include "pidll.h"
#include <cmath>

PIDController center;
PIDController left;
PIDController right;
PIDArchi archi; 

float kP_right=1;
float kD_right=0; 
float kI_right=0; 
float kP_left=1;
float kD_left=0;
float kI_left=0;
float kP_center=1;
float kD_center=0;
float kI_center=0;


void initPID(PIDController* PID, float* Kp, float* Ki, float* Kd) {
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->error = 0.f;
    PID->derivative = 0.f;
    PID->integral = 0.f;
}

void resetPID(PIDController* PID) {
    PID->error = 0.f;
    PID->derivative = 0.f;
    PID->integral = 0.f;
}

void updatePID(PIDController* PID, float error) {
    PID->derivative = error - PID->error;
    PID->integral += error;
    PID->error = error;
 
}

double getPIDOutput(PIDController* PID) {
    return (*PID->Kp) * PID->error + (*PID->Ki) * PID->integral + (*PID->Kd) * PID->derivative;
}

void initArchi(PIDArchi* ARCH, PIDController* left, PIDController* right, PIDController* central) {
    ARCH->left = left;
    ARCH->right = right;
    ARCH->central = central;
};


void resetArchi(PIDArchi* ARCH, double left_command, double right_command) {
    resetPID(ARCH->left);
    resetPID(ARCH->right);
    resetPID(ARCH->central);
    ARCH->command_left = left_command;
    ARCH->command_right = right_command;
    leftcounter=0;
    rightcounter=0;
    ARCH->cnt = 0;
    ARCH->limit = 70;
    ARCH->maxSpeed = 2000;
    ARCH->slowDownThreshold = 400;
};


void updateArchi(PIDArchi* ARCH, double left, double right) {
    updatePID(ARCH->left, ARCH->command_left - left);
    updatePID(ARCH->right, ARCH->command_right - right);
    updatePID(ARCH->central, left / ARCH->command_left - right / ARCH->command_right);
    ARCH->cnt += ARCH->cnt < ARCH->limit;
    ARCH.leftSlowDown = (abs(ARCH->left->error) < ARCH->slowDownThreshold) ? ARCH->left->error / ARCH->command_left : 1.;
    ARCH.rightSlowDown = (abs(ARCH->right->error) < ARCH->slowDownThreshold) ? ARCH->right->error / ARCH->command_right : 1.;
};

double getArchiLeftOutput(PIDArchi* ARCH) {
    double res = (getPIDOutput(ARCH->left) + ((ARCH->command_left > 0) ? -1 : 1 ) * getPIDOutput(ARCH->central)) * ARCH->cnt / ARCH->limit* ARCH->leftSlowDown;
    if (abs(res) < ARCH->maxSpeed) return res;
    if (res < 0) return -ARCH->maxSpeed;
    return ARCH->maxSpeed;
};

double getArchiRightOutput(PIDArchi* ARCH) {
    double res = (getPIDOutput(ARCH->right) + ((ARCH->command_right > 0) ? 1 : -1 ) * getPIDOutput(ARCH->central)) * ARCH->cnt / ARCH->limit * ARCH->rightSlowDown;
    if (abs(res) < ARCH->maxSpeed) return res;
    if (res < 0) return -ARCH->maxSpeed;
    return ARCH->maxSpeed;
}
