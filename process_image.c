#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

struct line {
	bool exist;
	uint16_t start;
	uint16_t end;
	uint16_t pos;
	uint16_t meanval;
};

struct obstacle{
	uint8_t type;
	uint16_t pos;
};

static struct line current_lines[MAX_LINES];
static struct obstacle current_obstacles[MAX_LINES];


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

void clear_all_lines(void)
{
	for(uint8_t i = 0 ; i < MAX_LINES ; i++)
	{
		current_lines[i].exist=FALSE;
		current_lines[i].start=0;
		current_lines[i].end=0;
		current_lines[i].pos=0;
	}
}

void clear_all_obstacles(void)
{
	for(uint8_t i = 0 ; i < MAX_LINES ; i++)
	{
		current_obstacles[i].type=0;
		current_obstacles[i].pos=0;
	}
}

void clear_obstacle(uint8_t i)
{
	current_obstacles[i].type=0;
	current_obstacles[i].pos=0;
}

void extract_lines(uint8_t *buffer)
{
	uint8_t stop = 0, falspos = 0, line_index = 0;
	uint16_t maxval=0;
	uint32_t mean = 0, localmean = 0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE-MIN_LINE_WIDTH/2 ; i+=MIN_LINE_WIDTH/2)
	{
		localmean = 0;
		for(uint16_t k = i ; k < i+MIN_LINE_WIDTH/2; k++)
		{
			localmean += buffer[k];
		}
		localmean /= MIN_LINE_WIDTH/2;
		if (localmean>maxval){maxval=localmean;}

	}

	if(maxval<STATIC_NOISE){return;}
	mean = maxval*RATIO_TO_QUALIFY_LINES;

	//clears the line array
	clear_all_lines();

	uint16_t i = 0;
	while (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_index<MAX_LINES)
	{
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
		    {
		    	falspos = 0;
		    	for(uint16_t k = i+WIDTH_SLOPE ; k < i + MIN_LINE_WIDTH ; k++)
		    	{
		    		if (buffer[k]<mean){falspos=1; break;}
		    	}
		    	if (falspos==0){
		        current_lines[line_index].start = i;
		        stop = 1;
		    	}
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) )
		{
		    stop = 0;
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] < mean && buffer[i-WIDTH_SLOPE] > mean && (i-current_lines[line_index].start)>MIN_LINE_WIDTH)
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


void extract_edges(uint8_t *buffer)
{

	uint8_t edge_index=0;
	uint16_t left_mean = 0, right_mean = 0;

	clear_all_obstacles();

	for(uint8_t i = 0 ; i < MAX_LINES ; i++)
	{
		if(current_lines[i].exist)
		{
			for(uint16_t j = 0 ; j < MIN_LINE_WIDTH && (current_lines[i].start-WIDTH_SLOPE-j)>0 && (current_lines[i].end+WIDTH_SLOPE+j)<IMAGE_BUFFER_SIZE; j++)
			{
				left_mean += buffer[current_lines[i].start-WIDTH_SLOPE-j];
				right_mean += buffer[current_lines[i].end+WIDTH_SLOPE+j];
			}
			left_mean /= MIN_LINE_WIDTH;
			right_mean /= MIN_LINE_WIDTH;

			if(left_mean < current_lines[i].meanval && right_mean < current_lines[i].meanval)
			{
				if(right_mean > RATIO_TO_QUALIFY_EDGES*left_mean && right_mean > RATIO_TO_CONFIRM_EDGES*current_lines[i].meanval)
				{
					current_obstacles[edge_index].type = RIGHT_EDGE;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
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


void extract_gate(void)
{
	for(uint8_t i = 0 ; i < MAX_LINES-1 ; i++)
	{
		if(current_obstacles[i].type==RIGHT_EDGE && current_obstacles[i+1].type==LEFT_EDGE)
		{
			current_obstacles[0].type = GATE;
			current_obstacles[0].pos = (current_obstacles[i].pos + current_obstacles[i+1].pos)/2;
			break;
		}
	}
}

//void choose_edge(void)
//{
//	uint8_t closer_edge = 0;
//	uint16_t dist_closer_edge =IMAGE_BUFFER_SIZE/2,  dist_index_edge;
//	if(current_obstacles[0].type!=GATE && current_obstacles[0].type!=GOAL)
//	{
//		for(uint8_t i = 0 ; i < MAX_LINES ; i++)
//		{
//			if(current_obstacles[i].type==RIGHT_EDGE || current_obstacles[i].type==LEFT_EDGE)
//			{
//				dist_index_edge = abs(current_obstacles[i].pos-IMAGE_BUFFER_SIZE/2);
//				if(dist_index_edge<dist_closer_edge)
//				{
//					dist_closer_edge = dist_index_edge;
//					closer_edge = i;
//				}
//			}
//		}
//		if(closer_edge)
//		{
//			current_obstacles[0].type=current_obstacles[closer_edge].type;
//			current_obstacles[0].pos=current_obstacles[closer_edge].pos;
//		}
//	}
//}


void extract_goal(void)
{
	uint8_t line_count = 0;
	for(uint8_t i = 0 ; i < MAX_LINES-1; i++)
	{
		if(current_lines[i].exist){line_count++;}
	}
	if (line_count > MIN_LINES_FOR_GOAL)
	{
	current_obstacles[0].type = GOAL;
	current_obstacles[0].pos = current_obstacles[1].pos;
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	bool send_to_computer = TRUE;

	clear_all_lines();
	clear_all_obstacles();

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
		{
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image_red[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}


		//search for a line in the image and gets its width in pixels
		extract_lines(image_red);
		extract_edges(image_red);
		extract_gate();
		//choose_edge();
		extract_goal();

		//TESTING
		for(uint8_t i = 0 ; i < MAX_LINES; i++)
		{
			if (current_obstacles[i].type != 0 && current_obstacles[i].type != UNKNOWN)
			{
				//chprintf((BaseSequentialStream *) &SD3, "(OBSTACLE_INDEX) = %d   TYPE: %d    POS:%d   \r\n", i, current_obstacles[i].type, current_obstacles[i].pos);
			}
		}

		if (send_to_computer)
		{
			//SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;
    }
}

uint8_t get_obstacle_type(void){
	return current_obstacles[0].type;
}

uint16_t get_obstacle_pos(void){
	return current_obstacles[0].pos;
}



void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}
