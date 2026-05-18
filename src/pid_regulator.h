#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

/**
 * @brief PID control constants for distance and rotation
 */
#define ROTATION_THRESHOLD		10      // px - minimal shift to trigger rotation correction
#define ROTATION_COEFF			1.2f 
#define ERROR_THRESHOLD			1.0f	//[mm] because of the noise of the camera

//PID gains tuned for mm
#define KP						45.0f   
#define KI 						0.1f	
#define KD                      0.4f    



#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)



void pid_regulator_start(void);


#endif /* PID_REGULATOR_H */