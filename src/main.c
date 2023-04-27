#include "hardware/irq.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include <pico/stdlib.h>
#include <com.h>
#include <stepper.h>
#include <motorpumpsvalve.h>
#include <pid.h>
#include <pidll.h>
#include <stdio.h> 


int main(){
//	uartInit();
//	uartIrqSetup();	
	stdio_init_all();
	while (1) {
		receive();
	}
}


