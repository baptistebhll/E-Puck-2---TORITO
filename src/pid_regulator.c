#include <ch.h>
#include <math.h>
#include <motors.h>
#include <stdlib.h>
#include <msgbus/messagebus.h>

#include "pid_regulator.h"
#include "process_image.h"
#include "robot_management.h"
#include "main.h"

static float sum_error = 0;
static float last_error = 0;

// PI regulator for camera-based distance control
int16_t pid_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	error = distance - goal;
	float derivative = error - last_error;

	last_error = error;

	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error + KD * derivative;

    return (int16_t)speed;
}



static THD_WORKING_AREA(waPIDRegulator, 256);
static THD_FUNCTION(PIDRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

	messagebus_topic_t *robot_state_topic = messagebus_find_topic_blocking(&bus,"/robot_state");
	robot_state_t current_robot_state;
	robot_state_t previous_robot_state = SEARCHING_RED;
	
    while(1){
        time = chVTGetSystemTime();

		
		messagebus_topic_read(robot_state_topic, &current_robot_state, sizeof(current_robot_state));

		// Reset PID errors if robot state changed
        if (current_robot_state != previous_robot_state) {
            sum_error = 0;
            last_error = 0;
            previous_robot_state = current_robot_state;
        }

		
		if (current_robot_state == CHASING_RED){ //PID activated only if we are CHASING_RED
			speed = pid_regulator(get_distance_mm(), GOAL_DISTANCE);
			speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

			if(abs(speed_correction) < ROTATION_THRESHOLD){
				speed_correction = 0;
			}
			right_motor_set_speed(speed - (int16_t)(ROTATION_COEFF * speed_correction));
			left_motor_set_speed(speed + (int16_t)(ROTATION_COEFF * speed_correction));
		}
		//100 Hz pour avoir un peu de marge
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void) {
	chThdCreateStatic(waPIDRegulator, sizeof(waPIDRegulator), NORMALPRIO, PIDRegulator, NULL);
}



