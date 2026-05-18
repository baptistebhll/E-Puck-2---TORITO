#ifndef ROBOT_MANAGEMENT_H
#define ROBOT_MANAGEMENT_H

#include <stdbool.h>           
#include <msgbus/messagebus.h>

typedef enum {
    SEARCHING_RED,
    CHASING_RED,
    AVOID_OBSTACLE
} robot_state_t;

typedef enum {
    SUB_ALIGN,          
    SUB_TURN_OUT,       
    SUB_MOVE_ALONG,     
    SUB_MOVE_CLEARANCE, 
    SUB_TURN_IN,        
    SUB_MOVE_SIDE,      
    SUB_TURN_IN2,
    SUB_MOVE_SIDE2,
    SUB_TURN_OUT2,
    SUB_MOVE_FINAL,      
    SUB_DONE            
} avoid_substate_t;

extern messagebus_topic_t robot_state_topic;


#define CM_PER_STEP                     (9.0f / 675.0f)     // = 0.01333f cm/step
#define GYRO_INTEGRATION_TIME           0.01f               //supposed to be 0.004 (4 ms) IMUs because sample rate = 250Hz but here we adapt to IR sample rate (100Hz)
#define OBSTACLE_CLEARED_DISTANCE       4.5f                // distance to make sure the robot is cleared of the obstacle
#define OBSTACLE_CONFIRMATION_THRESHOLD 5
#define PI                              3.14159f            
#define QUARTER_ROTATION                (PI / 2.0f)         // 90 degrees
#define ROTATION_SPEED_LIMIT            (MOTOR_SPEED_LIMIT / 3.0f) 
#define SQUARE_SIDE                     6.0f                // obstacles are cubic with a side of 6 cm


//raw values of get_calibrated_prox() for 1cm distance to detect obstacle
#define IR1_RAW_1CM                     245.0f      // Front Right 
#define IR8_RAW_1CM                     550.0f      // Front Left  

//raw value of get_calibrated_prox(2) for 2cm distance
#define IR3_RAW_2CM                     120.0f     // Side Right   

 
//We want get_norm_prox(i) to give us ~100 when we are 1cm from the object for sensors 1 and 8, 
// and when we are 2cm from the object for sensor 3
#define IR1_GAIN                        (100.0f / IR1_RAW_1CM)  // ~0.408
#define IR8_GAIN                        (100.0f / IR8_RAW_1CM)  // ~0.182
#define IR3_GAIN                        (100.0f / IR3_RAW_2CM)  // ~0.833


#define PROX_THRESHOLD_DETECTION        100.0f  // Threshold to go in AVOID_OBSTACLE (2cm)
#define PROX_PRECISION_ALIGNMENT        15.0f   // Threshold to be perpendicular to the obstacle

bool obstacle_detected(void);
float get_norm_prox(uint8_t sensor_index);
void robot_management_start(void);

#endif