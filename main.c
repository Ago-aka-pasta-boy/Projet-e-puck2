/*

File    : main.c
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Main file containing the FSM controlling the Homing Audio Localization (H.A.L.) robot project

Adapted from the code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <camera/po8030.h>
#include <motors.h>

#include <pathing.h>
#include <process_image.h>
#include <audio_processing.h>

//FSM control variables
static bool move_forward = 0;
static uint8_t obstacle_type = 0;

int main(void)
{
	//System and OS initializations
    halInit();
    chSysInit();
    mpu_init();

    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //inits the TOF sensor
    VL53L0X_start();
    //inits the Camera
    dcmi_start();
    po8030_start();

    //starts the image processing&capturing threads
    process_image_start();

    //starts the microphones processing thread, calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);

    //Sets the main thread's priority above the processing threads
    chThdSetPriority(NORMALPRIO +2);


    //-------------------------------------------FSM--------------------------------------------------------------

    //Main FSM loop
    while (1)
    {
    	obstacle_type=0;
    	if (!move_forward)
    	{
    		obstacle_type=rotate_to_source();
    	}
    	if (!obstacle_type)
    	{
    		obstacle_type=path_to_obstacle();
    	}
    	move_forward = FALSE;

    	if(obstacle_type)
    	{
			switch (obstacle_type)
			{
				case LEFT_EDGE:
					move_around_edge();
					move_forward = TRUE;
					break;

				case RIGHT_EDGE:
					move_around_edge();
					move_forward = TRUE;
					break;

				case GATE:
					move_through_gate();
					break;

				case GOAL:
					move_to_goal();
					break;

				case UNKNOWN:
					move_back();
					break;
			}
    	}
    }
}

//Stack Guard
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
