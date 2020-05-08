#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static struct line current_lines[MAXLINES];
static struct obstacle current_obstacles[MAXLINES];


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

void extract_lines(uint8_t *buffer){



	uint8_t stop = 0, falspos = 0, line_index = 0;
	uint16_t i = 0,  k = 0, maxval=0;
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

	//chprintf((BaseSequentialStream *) &SD3, "(MAXVAL) = %d \r\n", maxval);

	if(!maxval){return;}
	mean = maxval*0.7;

	//clears the line array
	for(uint16_t i = 0 ; i < MAXLINES ; i++)
	{
		current_lines[i].exist=FALSE;
		current_lines[i].start=0;
		current_lines[i].end=0;
		current_lines[i].width=0;
	}



	i = 0;
	while (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && line_index<MAXLINES)
	{
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
		    {
		    	falspos = 0;
		    	for(uint16_t k = i+WIDTH_SLOPE ; k < i + MIN_LINE_WIDTH ; k++){
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
		        	current_lines[line_index].pos = (current_lines[line_index].start + current_lines[line_index].end)/2;
		        	current_lines[line_index].width = current_lines[line_index].end - current_lines[line_index].start;
		        	current_lines[line_index].meanval = (buffer[current_lines[line_index].start+WIDTH_SLOPE] + buffer[current_lines[line_index].end-WIDTH_SLOPE])/2;
		        	line_index++;

		            stop = 1;
		        }
		        i++;
		    }
		    stop =0;
		}


	}

	for(uint16_t i = 0 ; i < MAXLINES ; i++){
		//chThdSleepMilliseconds(500);
		//chprintf((BaseSequentialStream *) &SD3, "(INDEX) = %d   START: %d    END:%d   MEANVAL: %d \r\n", i, current_lines[i].start, current_lines[i].end, current_lines[i].meanval);
		}

}


void extract_edges(uint8_t *buffer)
{

	uint8_t edge_index=0;
	uint16_t i = 0,  j = 0, left_mean = 0, right_mean = 0;

	for(uint16_t i = 0 ; i < MAXLINES ; i++)
	{
		current_obstacles[i].type=0;
		current_obstacles[i].pos=0;
		current_obstacles[i].width=0;
	}


	for(uint16_t i = 0 ; i < MAXLINES ; i++)
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
				if(left_mean > 2*right_mean && left_mean > 0.3*current_lines[i].meanval)
				{
					current_obstacles[edge_index].type = LEFT_EDGE;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
				else if(right_mean > 2*left_mean && right_mean > 0.3*current_lines[i].meanval)
				{
					current_obstacles[edge_index].type = RIGHT_EDGE;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
				else
				{
					current_obstacles[edge_index].type = UNKNOWN;
					current_obstacles[edge_index].pos = current_lines[i].pos;
				}
				edge_index++;
			}
			//chprintf((BaseSequentialStream *) &SD3, "LINE: %d       (LEFT_MEAN) = %d   (RIGHT_MEAN): %d  \r\n", i, left_mean, right_mean);
		}
	}

}


void extract_gate(void)
{
	for(uint16_t i = 0 ; i < MAXLINES-1 ; i++)
	{
		if(current_obstacles[i].type==RIGHT_EDGE && current_obstacles[i+1].type==LEFT_EDGE)
		{
			current_obstacles[i].type = GATE;
			current_obstacles[i].pos = (current_obstacles[i].pos + current_obstacles[i+1].pos)/2;
			clear_obstacle(i+1);
			break;
		}
	}
}

void choose_edge(void)
{
	uint8_t closer_edge = 0;
	uint16_t dist_closer_edge =IMAGE_BUFFER_SIZE/2,  dist_index_edge;
	if(current_obstacles[0].type!=GATE && current_obstacles[0].type!=GOAL)
	{
		for(uint8_t i = 0 ; i < MAXLINES ; i++)
		{
			if(current_obstacles[i].type==RIGHT_EDGE || current_obstacles[i].type==LEFT_EDGE)
			{
				dist_index_edge = abs(current_obstacles[i].pos-IMAGE_BUFFER_SIZE/2);
				if(dist_index_edge<dist_closer_edge)
				{
					dist_closer_edge = dist_index_edge;
					closer_edge = i;
				}
			}
		}
		if(closer_edge)
		{
			current_obstacles[0].type=current_obstacles[closer_edge].type;
			current_obstacles[0].pos=current_obstacles[closer_edge].pos;
		}
	}
}


void extract_goal(void)
{
	for(uint8_t i = 0 ; i < MAXLINES-2; i++)
	{
		if(!current_obstacles[i].type){return;}
	}
	current_obstacles[0].type = GOAL;
	current_obstacles[0].pos = current_obstacles[1].pos;

	for(uint16_t i = 1 ; i < MAXLINES; i++){clear_obstacle(i);}
}

void clear_obstacle(uint8_t i)
{
	current_obstacles[i].type=0;
	current_obstacles[i].pos=0;
	current_obstacles[i].width=0;
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	//uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
	//uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = TRUE;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image_red[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
			//image_green[i/2] = ((((uint8_t)img_buff_ptr[i]&0x07)<<5)|(((uint8_t)img_buff_ptr[i+1]&0xE0>>3)))/2;
			//image_blue[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F)<<3;
		}



		//search for a line in the image and gets its width in pixels
		extract_lines(image_red);
		extract_edges(image_red);
		extract_gate();
		extract_goal();
		choose_edge();


		//TESTING
		for(uint16_t i = 0 ; i < MAXLINES; i++)
		{
			if (current_obstacles[i].type != 0 && current_obstacles[i].type != UNKNOWN)
			{
				chprintf((BaseSequentialStream *) &SD3, "(OBSTACLE_INDEX) = %d   TYPE: %d    POS:%d   \r\n", i, current_obstacles[i].type, current_obstacles[i].pos);
			}
		}

		//chprintf((BaseSequentialStream *) &SD3, "image thread    \r\n");



		if (send_to_computer)
		{
			//SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
		}

		send_to_computer = !send_to_computer;
    }
}

uint8_t get_obstacle_type(uint8_t index){
	return current_obstacles[index].type;
}

uint16_t get_obstacle_pos(uint8_t index){
	return current_obstacles[index].pos;
}



void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
