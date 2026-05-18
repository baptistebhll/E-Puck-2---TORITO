#include <ch.h>
#include <hal.h>
#include <stdbool.h>            
#include <stdint.h>             
#include <memory_protection.h>  
#include <usbcfg.h>

#include <motors.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>

#include "main.h"
#include "pid_regulator.h"
#include "process_image.h"
#include "robot_management.h"


messagebus_t bus; 
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


int main(void)
{
    halInit();
    chSysInit();
	mpu_init();
	usb_start(); 

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	dcmi_start();
	po8030_start();
	po8030_set_awb(0); //disables auto white balance 
	motors_init();
	proximity_start();
	calibrate_ir();
	imu_start();

	
	pid_regulator_start();
	process_image_start();	
	robot_management_start();  

	
	while(true) {
		chThdSleepMilliseconds(1000);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
