#include <ch.h>
#include <hal.h>
#include <camera/po8030.h>
#include <stdint.h>
#include <stdbool.h>


#include "process_image.h"
#include "robot_management.h"
#include "main.h"

static uint16_t distance_mm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static bool red_detected_check = 0;



//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOMM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean) //si RISING EDGE (fond noir), sinon FALLING edge (fond blanc) ->inverser les inégalités
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] < mean && buffer[i-WIDTH_SLOPE] > mean) //si RISING EDGE (fond noir), sinon FALLING edge (fond blanc) ->inverser les inégalités
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOMM/width) > MAX_DISTANCE){
		return PXTOMM/MAX_DISTANCE;
	}else{
		return width;
	}
}




static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the lines USED_LINE and USED_LINE + 1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, USED_LINE, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImage, 1000);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0}; 
	uint16_t nbr_red_pixel = 0;	
	uint16_t lineWidth = 0;
	uint32_t red_mean = 0;

	messagebus_topic_t *robot_state_topic = messagebus_find_topic_blocking(&bus,"/robot_state");
    robot_state_t current_robot_state;  


    while(1){
		messagebus_topic_read(robot_state_topic, &current_robot_state, sizeof(current_robot_state));

		//gets the pointer to the array filled with the last image in RGB565  
		img_buff_ptr = dcmi_get_last_image_ptr();
		
		//waits until an image has been captured
        chBSemWait(&image_ready_sem);


		for (uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts the red of the image and fills buffer
			image_r[i/2] = (uint8_t)img_buff_ptr[i] & 0xF8;
		}
		
		for (uint16_t j = 0; j < IMAGE_BUFFER_SIZE; j++) { 
			if (image_r[j] > RED_PIXEL_INTENSITY_THRESHOLD) {
				red_mean += image_r[j];
				nbr_red_pixel++;
			}
		}
		if (nbr_red_pixel){
			red_mean /= nbr_red_pixel;
		}
		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image_r);

		red_detected_check = (lineWidth >= MIN_LINE_WIDTH) && (red_mean > RED_PIXEL_INTENSITY_THRESHOLD);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_mm = PXTOMM/lineWidth;
		}

		nbr_red_pixel = 0;
		red_mean = 0;
    }
}

uint16_t get_distance_mm(void){
	return distance_mm;
}

uint16_t get_line_position(void){
	return line_position;
}
bool get_red_detected(void){
	return red_detected_check;
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

