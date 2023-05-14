#include "pid.h"
#include "pidll.h"
PIDController distance;
PIDController direction;
PIDArchi archi; 


float kprot=14;
float kdrot=2;
float kirot=0;
float kptrans=6;
float kdtrans=1;
float kitrans=5;


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
	for (int i = 0; i < 100; i++) PID->memory[i] = 0.;
}

void updatePID(PIDController* PID, float error) {
    PID->derivative = (error - PID->error) / 0.005;	
	PID->integral += (error - PID->memory[0])* 0.005;
    PID->error = error;
	for (int i=0; i<99; i++) PID->memory[i] = PID->memory[i+1];
	PID->memory[99] = error;
}

double getPIDOutput(PIDController* PID) {
    return (*PID->Kp) * PID->error + (*PID->Ki) * PID->integral + (*PID->Kd) * PID->derivative;
}

void initArchi(PIDArchi* ARCH, PIDController* distance, PIDController* direction) {
    ARCH->distance = distance;
    ARCH->direction = direction;
};


void resetArchi(PIDArchi* ARCH, double left_command, double right_command) {
    resetPID(ARCH->distance);
    resetPID(ARCH->direction);
    ARCH->command_distance = .5 * (left_command + right_command);
    ARCH->command_direction = .5 * (right_command - left_command);
	ARCH->cnt = 0;
	ARCH->limit = 120;
	leftcounter=0;
	rightcounter=0;
};


void updateArchi(PIDArchi* ARCH, double left, double right) {
    updatePID(ARCH->distance, ARCH->command_distance - .5 * (left + right));
    updatePID(ARCH->direction, ARCH->command_direction - .5 * (right - left));
	ARCH->cnt += ARCH->cnt < ARCH->limit;
};

double getArchiLeftOutput(PIDArchi* ARCH) {
    return (getPIDOutput(ARCH->distance) - 3*getPIDOutput(ARCH->direction)) * ARCH->cnt / ARCH->limit;
};

double getArchiRightOutput(PIDArchi* ARCH) {
    return (getPIDOutput(ARCH->distance) + 3*getPIDOutput(ARCH->direction)) * ARCH->cnt / ARCH->limit;
};
