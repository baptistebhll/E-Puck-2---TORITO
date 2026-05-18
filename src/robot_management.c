#include <ch.h>
#include <math.h>
#include <stdlib.h>

#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <motors.h>

#include "main.h"
#include "robot_management.h"
#include "process_image.h"  




bool obstacle_detected(void) {
    static uint8_t obstacle_counter = 0; //keep the counter between the calls
    float norm_ir1 = get_norm_prox(0);
    float norm_ir8 = get_norm_prox(7);

    if (norm_ir1 > PROX_THRESHOLD_DETECTION || norm_ir8 > PROX_THRESHOLD_DETECTION) {
        obstacle_counter++;
    } else {
        obstacle_counter = 0;
    }
    return (obstacle_counter >= OBSTACLE_CONFIRMATION_THRESHOLD);
}

const float ir_gains[8] = {
    IR1_GAIN, 
    1.0f,     
    IR3_GAIN, 
    1.0f,     
    1.0f,     
    1.0f,     
    1.0f,     
    IR8_GAIN  
};

float get_norm_prox(uint8_t sensor_index) {
    if (sensor_index >= 8) return 0;
    return (float)get_calibrated_prox(sensor_index) * ir_gains[sensor_index]; 
}

static THD_WORKING_AREA(waRobot, 500);
static THD_FUNCTION(Robot, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    float distance = 0; 
    int32_t ref_left  = 0;
    int32_t ref_right = 0;
    int32_t steps_l = 0;
    int32_t steps_r = 0;
    float  total_rotation = 0;     

    uint8_t current_robot_state = SEARCHING_RED;  
    avoid_substate_t current_sub_state = SUB_ALIGN;

    MUTEX_DECL(robot_topic_lock);
    CONDVAR_DECL(robot_topic_condvar);
    
    messagebus_topic_t robot_state_topic;
    messagebus_topic_init(&robot_state_topic, &robot_topic_lock, &robot_topic_condvar, 
                          &current_robot_state, sizeof(current_robot_state));
    messagebus_advertise_topic(&bus, &robot_state_topic, "/robot_state");
    


    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus,"/proximity");
    proximity_msg_t proximity_msg;    
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus,"/imu");
    imu_msg_t imu_values;


    messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));

    while(1){
        messagebus_topic_wait(proximity_topic, &proximity_msg, sizeof(proximity_msg)); //100 Hz
        messagebus_topic_read(imu_topic, &imu_values, sizeof(imu_values)); //IMUs are sampled at 250 Hz but we adapt to the proximity topic rate (the slowest)

        switch(current_robot_state) {
            case SEARCHING_RED:
                left_motor_set_speed(ROTATION_SPEED_LIMIT);
                right_motor_set_speed(-ROTATION_SPEED_LIMIT);

                if (obstacle_detected()) {
                    current_sub_state = SUB_ALIGN;
                    current_robot_state = AVOID_OBSTACLE;
                    messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));
                }else if (get_red_detected()) {
                    current_robot_state = CHASING_RED;
                    messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));
                }
                break;
                
            case CHASING_RED:
                if(obstacle_detected()) { 
                    current_sub_state = SUB_ALIGN;
                    current_robot_state = AVOID_OBSTACLE;
                    messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));
                }else if (!get_red_detected()) {
                    current_robot_state = SEARCHING_RED;
                    messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));
                }
                break;
                
            case AVOID_OBSTACLE:
                switch(current_sub_state) {
                    case SUB_ALIGN:
                        if (abs(get_norm_prox(7) - get_norm_prox(0)) > PROX_PRECISION_ALIGNMENT) {
                            if (get_norm_prox(7) > get_norm_prox(0)) {
                                //obstacle on the left, turn slowly to the left
                                left_motor_set_speed(-ROTATION_SPEED_LIMIT / 2); 
                                right_motor_set_speed(ROTATION_SPEED_LIMIT / 2); 
                            } else {
                                //obstacle on the right, turn slowly to the right
                                left_motor_set_speed(ROTATION_SPEED_LIMIT / 2); 
                                right_motor_set_speed(-ROTATION_SPEED_LIMIT / 2); 
                            }
                        } else {
                            total_rotation = 0;
                            current_sub_state = SUB_TURN_OUT;
                        }
                        break;
                    case SUB_TURN_OUT :
                        
                        if (total_rotation < QUARTER_ROTATION) {
                            left_motor_set_speed(-ROTATION_SPEED_LIMIT); 
                            right_motor_set_speed(ROTATION_SPEED_LIMIT);
                            total_rotation += fabs(imu_values.gyro_rate[Z_AXIS]) * GYRO_INTEGRATION_TIME;
                        } else {
                            current_sub_state = SUB_MOVE_ALONG;
                        }
                        
                        break;
                    case SUB_MOVE_ALONG: //move forward until IR3 can no longer see anything
                        if (obstacle_detected()) {
                            total_rotation = 0;
                            distance = 0;
                            current_sub_state = SUB_ALIGN; 
                        } else {
                            if (get_norm_prox(2) >= PROX_THRESHOLD_DETECTION) {
                                left_motor_set_speed(MOTOR_SPEED_LIMIT/2); 
                                right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
                            } else {
                                ref_left  = left_motor_get_pos();
                                ref_right = right_motor_get_pos();
                                current_sub_state = SUB_MOVE_CLEARANCE;
                            }
                        }
                        break;

                    case SUB_MOVE_CLEARANCE: //move forward of OBSTACLE_CLEARED_DISTANCE to be able to turn right
                        if (obstacle_detected()) {
                            total_rotation = 0;
                            distance = 0;
                            current_sub_state = SUB_ALIGN; 
                        } else {
                            steps_l  = left_motor_get_pos()  - ref_left;
                            steps_r  = right_motor_get_pos() - ref_right;
                            distance = ((steps_l + steps_r) / 2.0f) * CM_PER_STEP;

                            if (distance < OBSTACLE_CLEARED_DISTANCE) {
                                left_motor_set_speed(MOTOR_SPEED_LIMIT / 2); 
                                right_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                            } else {
                                total_rotation = 0;
                                current_sub_state = SUB_TURN_IN;
                            }
                        }
                        break;
                    
                    case SUB_TURN_IN: //turn 90° to the right
                        
                        if (total_rotation < QUARTER_ROTATION) {
                            left_motor_set_speed(ROTATION_SPEED_LIMIT); 
                            right_motor_set_speed(-ROTATION_SPEED_LIMIT);
                            total_rotation += fabs(imu_values.gyro_rate[Z_AXIS]) * GYRO_INTEGRATION_TIME;
                        } else {
                            ref_left  = left_motor_get_pos();
                            ref_right = right_motor_get_pos();
                            current_sub_state = SUB_MOVE_SIDE;
                        }
                    
                        break;
                    
                    case SUB_MOVE_SIDE: //move forward of the length of the cube plus 2*OBSTACLE_CLEARED_DISTANCE to be sure to be able to turn
                        if (obstacle_detected()) {
                            total_rotation = 0;
                            distance = 0;
                            current_sub_state = SUB_ALIGN; 
                        } else {
                            steps_l  = left_motor_get_pos()  - ref_left;
                            steps_r  = right_motor_get_pos() - ref_right;
                            distance = ((steps_l + steps_r) / 2.0f) * CM_PER_STEP;

                            if (distance < (SQUARE_SIDE + 2 * OBSTACLE_CLEARED_DISTANCE)) {
                                left_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                                right_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                            } else {
                                total_rotation = 0;
                                current_sub_state = SUB_TURN_IN2;
                            }
                        }
                        break;
                    
                    case SUB_TURN_IN2://turn 90° to the right
                       
                        if (total_rotation < QUARTER_ROTATION) {
                            left_motor_set_speed(ROTATION_SPEED_LIMIT); 
                            right_motor_set_speed(-ROTATION_SPEED_LIMIT);
                            total_rotation += fabs(imu_values.gyro_rate[Z_AXIS]) * GYRO_INTEGRATION_TIME;
                        } else {
                            ref_left  = left_motor_get_pos();
                            ref_right = right_motor_get_pos();
                            current_sub_state = SUB_MOVE_SIDE2;
                        }
                        
                        break;

                    case SUB_MOVE_SIDE2: //move forward of the length of the cube + 2*OBSTACLE_CLEARED_DISTANCE to be sure to be able to turn
                        if (obstacle_detected()) {
                            total_rotation = 0;
                            distance = 0;
                            current_sub_state = SUB_ALIGN; 
                        } else {
                            steps_l  = left_motor_get_pos()  - ref_left;
                            steps_r  = right_motor_get_pos() - ref_right;
                            distance = ((steps_l + steps_r) / 2.0f) * CM_PER_STEP;

                            if (distance < (SQUARE_SIDE / 2 + OBSTACLE_CLEARED_DISTANCE)) {
                                left_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                                right_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                            } else {
                                total_rotation = 0;
                                current_sub_state = SUB_TURN_OUT2;
                            }
                        }
                        break;

                    case SUB_TURN_OUT2://turn 90° to the left
                        
                        if (total_rotation < QUARTER_ROTATION) {
                                left_motor_set_speed(-ROTATION_SPEED_LIMIT); 
                                right_motor_set_speed(ROTATION_SPEED_LIMIT);
                                total_rotation += fabs(imu_values.gyro_rate[Z_AXIS]) * GYRO_INTEGRATION_TIME;
                        } else {
                                ref_left  = left_motor_get_pos();
                                ref_right = right_motor_get_pos();
                                current_sub_state = SUB_MOVE_FINAL;
                        }
                    
                        break;

                    case SUB_MOVE_FINAL: //moveforward of OBSTACLE_CLEARED_DISTANCE to be fully cleared from the obstacle
                        if (obstacle_detected()) {
                            total_rotation = 0;
                            distance = 0;
                            current_sub_state = SUB_ALIGN; 
                        } else {
                            steps_l  = left_motor_get_pos()  - ref_left;
                            steps_r  = right_motor_get_pos() - ref_right;
                            distance = ((steps_l + steps_r) / 2.0f) * CM_PER_STEP;

                            if (distance < OBSTACLE_CLEARED_DISTANCE) {
                                left_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                                right_motor_set_speed(MOTOR_SPEED_LIMIT / 2);
                            } else {
                                current_sub_state = SUB_DONE;
                            }
                        }
                        break;

                    case SUB_DONE:
                        left_motor_set_speed(0);
                        right_motor_set_speed(0);
                        current_sub_state = SUB_ALIGN; //reset of sub state
                        current_robot_state = SEARCHING_RED;
                        messagebus_topic_publish(&robot_state_topic, &current_robot_state, sizeof(current_robot_state));
                        break;
                }
                break;
                
            }
    }
}

void robot_management_start(void) {
    chThdCreateStatic(waRobot, sizeof(waRobot), NORMALPRIO, Robot, NULL);
}
