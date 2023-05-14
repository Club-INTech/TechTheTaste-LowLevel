#include <inttypes.h>
typedef struct{
	int id;
	int dirPin;
	int stepPin;
	int position;
	int dir;
	int target;
	int speed;
	int pwmSlice;
	int pwmChan;
	int axis; 
	}stepper; 

typedef struct{
	int axis;
	int pin;
	}endstop;


extern stepper steppers[8];

double clockDivider(int speed);
int wrapCalculator(int speed);
void endstoprise(uint gpio , uint32_t event_mask);
int homming(stepper *stepper, endstop *endstop,int dir);
int endstopInit(endstop *endstop, int pin, int axis);
int maskMotor(stepper *motor);
void stepperCountDown();
int irqPwmSetup();
int irqStepperSetup(stepper *stepper);
int stepperInit(stepper *stepper,int id,int dirPin, int stepPin, int speed,int axis);
int home(stepper stepper[],endstop endstop[], int dir [],int orderlenght);
int motorValueStepper(stepper motors[],int id[], int dir[], int target[],int orderlenght);
int armMove(int target[],int speed);
extern int armstop;
