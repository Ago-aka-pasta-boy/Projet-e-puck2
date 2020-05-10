#include "ch.h"
#include "hal.h"
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

//Local defines
#define MAX_OBJECTS				5		//Maximum objects in the line and obstacle arrays

#define IMAGE_BUFFER_SIZE		640 	//Size of the image buffer where the red camera pixel data is stored
#define WIDTH_SLOPE				5		//Maximum width of the transition from below threshold to above for line extraction
#define MIN_LINE_WIDTH			40		//Minimum width to qualify as a line

#define LOCAL_MEAN_SIZE			20		//Size of the local segment means performed to compute a threshold value
#define STATIC_NOISE			100   	//If no local means are above this value, no lines are registered
#define	RATIO_TO_QUALIFY_LINES	0.7		//Ratio to max local mean to compute threshold value
#define RATIO_TO_QUALIFY_EDGES	2		//Bright side means must register at least twice as high as dark side means for edges
#define RATIO_TO_CONFIRM_EDGES	0.3		//The higher of the two means for edge deduction must be at least 0.3 times the white line
#define MIN_LINES_FOR_GOAL		3		//Minimum number to confirm a goal

//Line structure
struct line {
	bool exist;
	uint16_t start;
	uint16_t end;
	uint16_t pos;
	uint16_t meanval;
};

//Obstacle structure
struct obstacle{
	uint8_t type;
	uint16_t pos;
};

//The static arrays containing all registered lines and obstacles respectively.
static struct line current_lines[MAX_OBJECTS];
static struct obstacle current_obstacles[MAX_OBJECTS];

//Semaphore for alerting the process image thread when a new image is ready from the capture image thread
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


//Clears all lines
void clear_all_lines(void)
{
	for(uint8_t i = 0 ; i < MAX_OBJECTS ; i++)
	{
		current_lines[i].exist=FALSE;
		current_lines[i].start=0;
		current_lines[i].end=0;
		current_lines[i].pos=0;
	}
}


//Clears all obstacles
void clear_all_obstacles(void)
{
	for(uint8_t i = 0 ; i < MAX_OBJECTS ; i++)
	{
		current_obstacles[i].type=0;
		current_obstacles[i].pos=0;
	}
}


//Clears a single obstacle index in the array
void clear_obstacle(uint8_t i)
{
	current_obstacles[i].type=0;
	current_obstacles[i].pos=0;
}


//Extracts lines from the camera data. Used as the basis for obstacle recognition
void extract_lines(uint8_t *buffer)
{
	uint8_t stop = 0, false_pos = 0, line_index = 0;
	uint16_t max_mean=0;
	uint32_t threshold = 0, local_mean = 0;

	clear_all_lines();

	//Compute local means on segments of the image buffer of size LOAL_MEAN_SIZE and keep the maximum local mean
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE-LOCAL_MEAN_SIZE ; i+=LOCAL_MEAN_SIZE)
	{
		local_mean = 0;
		for(uint16_t k = i ; k < i+LOCAL_MEAN_SIZE; k++)
		{
			local_mean += buffer[k];
		}
		local_mean /= LOCAL_MEAN_SIZE;
		if (local_mean>max_mean){max_mean=local_mean;}
	}

	//Checks if the maximum mean is above a certain value to reject "noise lines" in low contrast images
	if(max_mean<STATIC_NOISE){return;}
	//Sets a threshold value for line searching
	threshold = max_mean*RATIO_TO_QUALIFY_LINES;


	uint16_t i = 0;
	//Searches through the image buffer for line begins and ends until the end is reached or the line array is full
	while (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_index<MAX_OBJECTS)
	{
		//Finds the starting point of a line
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//Searches for a transition from below to above threshold within WIDTH_SLOPE pixels
		    if(buffer[i] < threshold && buffer[i+WIDTH_SLOPE] > threshold)
		    {
		    	//Ensures the value does not fall below threshold before MIN_LINE_WIDTH pixels
		    	false_pos = 0;
		    	for(uint16_t k = i+WIDTH_SLOPE ; k < i + MIN_LINE_WIDTH ; k++)
		    	{
		    		if (buffer[k]<threshold){false_pos=1; break;}
		    	}
		    	if (false_pos==0){
		        current_lines[line_index].start = i;
		        stop = 1;
		    	}
		    }
		    i++;
		}

		//Checks if line begin was found
		if (current_lines[line_index].start )
		{
		    stop = 0;
		    //Finds the end of the line
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] < threshold && buffer[i-WIDTH_SLOPE] > threshold && (i-current_lines[line_index].start)>MIN_LINE_WIDTH)
		        {
		        	current_lines[line_index].exist = TRUE;
		        	current_lines[line_index].end = i;
		        	current_lines[line_index].pos = current_lines[line_index].end - current_lines[line_index].start;
		        	current_lines[line_index].meanval = (buffer[current_lines[line_index].start+WIDTH_SLOPE] + buffer[current_lines[line_index].end-WIDTH_SLOPE])/2;
		        	line_index++;

		            stop = 1;
		        }
		        i++;
		    }
		    stop =0;
		}
	}
}


//Extracts edges from the camera data and the lines in the array
void extract_edges(uint8_t *buffer)
{

	uint8_t edge_index=0;
	uint16_t left_mean = 0, right_mean = 0;

	clear_all_obstacles();

	//Computes the means on the left and right sides for each (existing) line in the array and compares them to deduce edge type
	//Left edges are encoded green->white_line->dark, while right edges are dark->white_line->green (from left to right)
	for(uint8_t i = 0 ; i < MAX_OBJECTS ; i++)
	{
		if(current_lines[i].exist)
		{
			for(uint16_t k = 0 ; k < MIN_LINE_WIDTH && (current_lines[i].start-WIDTH_SLOPE-k)>0 && (current_lines[i].end+WIDTH_SLOPE+k)<IMAGE_BUFFER_SIZE; k++)
			{
				left_mean += buffer[current_lines[i].start-WIDTH_SLOPE-k];
				right_mean += buffer[current_lines[i].end+WIDTH_SLOPE+k];
			}
			left_mean /= MIN_LINE_WIDTH;
			right_mean /= MIN_LINE_WIDTH;

			//Checks if both the left and right sides are of lower mean value than the white line
			if(left_mean < current_lines[i].meanval && right_mean < current_lines[i].meanval)
			{
				//Right mean must be RATIO_TO_QUALIFY times larger than the left mean and RATIO_TO_CONFIRM times smaller than the line -> right edge
				if(right_mean > RATIO_TO_QUALIFY_EDGES*left_mean && right_mean > RATIO_TO_CONFIRM_EDGES*current_lines[i].meanval)
				{
					current_obstacles[edge_index].type = RIGHT_EDGE;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
				//Left mean must be RATIO_TO_QUALIFY times larger than the right mean and RATIO_TO_CONFIRM times smaller than the line -> left edge
				else if(left_mean > RATIO_TO_QUALIFY_EDGES*right_mean && left_mean > RATIO_TO_CONFIRM_EDGES*current_lines[i].meanval)
				{
					current_obstacles[edge_index].type = LEFT_EDGE;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
				edge_index++;
			}
		}
	}
}


//Builds a gate from an adjacent right edge - left edge combination if present in the array
void build_gate(void)
{
	for(uint8_t i = 0 ; i < MAX_OBJECTS-1 ; i++)
	{
		if(current_obstacles[i].type==RIGHT_EDGE && current_obstacles[i+1].type==LEFT_EDGE)
		{
			current_obstacles[0].type = GATE;
			current_obstacles[0].pos = (current_obstacles[i].pos + current_obstacles[i+1].pos)/2;
			break;
		}
	}
}


//Builds a goal from the line data if enough are present in the array
void build_goal(void)
{
	uint8_t line_count = 0;
	for(uint8_t i = 0 ; i < MAX_OBJECTS-1; i++)
	{
		if(current_lines[i].exist){line_count++;}
	}
	if (line_count >= MIN_LINES_FOR_GOAL)
	{
	current_obstacles[0].type = GOAL;
	current_obstacles[0].pos = current_obstacles[1].pos;
	}
}


//Image capturing thread in charge of capturing the camera data and signaling the process image thread when the data is ready
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the lines 10 + 11 (needs a minimum of 2 lines)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //Starts a capture
		dcmi_capture_start();
		//Waits for the capture to be done
		wait_image_ready();
		//Signals an image has been captured to the process image thread
		chBSemSignal(&image_ready_sem);
    }
}


//Image processing thread in charge of preparing the raw data and creating lines and obstacles
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};

	//Initializes the arrays to zero
	clear_all_lines();
	clear_all_obstacles();

    while(1){
    	//Waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//Gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
		{
			//Extracts first 5bits of the first byte (RGB565 Format)
			image_red[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//Extracts lines from the camera data
		extract_lines(image_red);
		//Extracts edges from the camera data and the line array
		extract_edges(image_red);
		//Builds a gate from edges (if available)
		build_gate();
		//Build a  goal from lines (if available)
		build_goal();
    }
}


//Returns the type of the obstacle in the 0th index of the array, considered the currently seen obstacle, to external modules
uint8_t get_obstacle_type(void)
{
	return current_obstacles[0].type;
}


//Returns the position of the obstacle in the 0th index of the array, considered the currently seen obstacle, to external modules
uint16_t get_obstacle_pos(void)
{
	return current_obstacles[0].pos;
}


//Starts the image capture and image processing threads
void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}
