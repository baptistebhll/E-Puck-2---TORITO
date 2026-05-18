#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <stdint.h>  
#include <stdbool.h> 

// Specify the 2 consecutive lines used for tracking the black line
// The line number starts from 0 and ending to PO8030_MAX_HEIGHT - 1. Consult camera/po8030.h
// But as 2 lines will be used, the value of the first line can be higher than PO8030_MAX_HEIGHT - 2
// a value around 450 means the robot will look at the grond
#define USED_LINE 200   // Must be inside [0..478], according to the above explanations

// Constants related to the image acquisition and processing
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40 //pixels
#define PXTOMM					15700 //experimental value for a distance in mm
#define GOAL_DISTANCE 			70.0f //in mm
#define MAX_DISTANCE 			250.0f //in mm
#define RED_PIXEL_INTENSITY_THRESHOLD		200  //experimental




uint16_t get_distance_mm(void);
uint16_t get_line_position(void);
bool get_red_detected(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */