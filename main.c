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

#include "leds.h"
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <camera/po8030.h>

#include <sensors/proximity.h>
#include <msgbus/messagebus.h>

#include <pathing.h>
#include <process_image.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

static uint8_t obstacle_type = 0;

//Sends data to the computer for visualization and testing
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}


//Starts the serial communication
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


//General Purpose Timer configuration
static void timer12_start(void)
{
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}


//Short sign(x) function
int sign(float x){return (x > 0) - (x < 0);}



int main(void)
{
	//System and OS initializations
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
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

    //-------------------------------------------FSM--------------------------------------------------------------

    //FSM control variables
    bool move_forward = 0;

    float ang_=0, ang_f=0;
    int dis = 0;


//    while(1)
//    {
//
//    	chThdSleepMilliseconds(500);
//    	dis = get_prox(2);
//    	chprintf((BaseSequentialStream *) &SD3, "dis: %d    \r\n", dis);
//    	rotate_to_source();
//    	set_speed(700);
//
//    }
//
//while(1){chThdSleepMilliseconds(100);}

    //Main FSM loop
    while (1)
    {
    	obstacle_type=0;
    	if (!move_forward)
    	{
    	obstacle_type=rotate_to_source();
    	}
    	move_forward = FALSE;
    	obstacle_type=path_to_obstacle();
    	if(obstacle_type)
    	{
			switch (obstacle_type)
			{
				case LEFT_EDGE:
					move_around_edge();
					move_forward = TRUE;
					chThdSleepMilliseconds(1000);
					break;

				case RIGHT_EDGE:
					move_around_edge();
					move_forward = TRUE;
					chThdSleepMilliseconds(1000);
					break;

				case GATE:
					move_through_gate();
					chThdSleepMilliseconds(1000);
					break;

				case GOAL:
					move_to_goal();
					chThdSleepMilliseconds(1000);
					break;
			}
    	}
    }
    chprintf((BaseSequentialStream *) &SD3, "LEAVING MAIN    \r\n");
}

//Stack Guard
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
