/*
 * pathing.c
 *
 *  Created on: 3 May 2020
 *      Author: nicol
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <arm_math.h>

#include <sensors/VL53L0X/VL53L0X.h>
#include <pathing.h>
#include <motors.h>
#include <process_image.h>
#include <audio_processing.h>
#include "leds.h"

//Defines
#define ROT_COEF 				7			//Experimental value, ROT_COEF * angle gives the speed at which to turn during ROT_TIME
#define ROT_TIME				500			//[ms] Half a second is needed to have big angles
#define SAFETY_DISTANCE 		40			//[mm] The robot will go into reverse if he sees anything closer
#define MAX_TRAVEL_TIME			3000		//[ms] Maximum amount of time the robot will go forward without reorientating itself
#define MAX_ROT_TIME 			3000		//[ms] Maximum amount of time the robot will spend trying to orientate itself
#define COLLISION_DISTANCE 		30			//[mm] Distance at which the robot considers he collided with the gate he was ramming
#define MINIMUM_ROT_SPEED		175			//Turning at a lower speed starts to make the robot shake instead of turning properly
#define SLOW_DOWN_DISTANCE		200			//[mm] The robot will slow down at that distance to facilitate the obstacle recognition
#define SLOW_SPEED				300			//[step/s]
#define FAST_SPEED				500			//[step/s]
#define HALT					0			//[step/s]
#define	LARGE_ANGLE				40			//[degrees] Angle after which the past average of the filter will be reset to fix long convergence times
#define STABILIZATION_TRIES		5			//Maximum amount of times the angle is polled to determine its stability
#define STABILIZED_AUDIO		2			//The angle will be considered stable if more than 2 of the tries are accepted
#define MAX_ANGLE_ERROR			15.0f		//[degrees] The maximum error of the sound localization accuracy
#define RAM_SPEED				1000		//[step/s]
#define CELEBRATION_TIME		1000		//[ms]
#define TOO_CLOSE				80			//[mm] The robot will reach this distance when moving back
#define MAX_POS_SHIFT			50			//[nb of pixels] The maximum accepted difference of position between two instances of the same obstacle
#define MAX_DIST_TO_CONSIDER	100			//[mm] Obstacles further than this will not be taken into consideration
#define THRESHOLD_EDGE_AND_GOAL 4			//Count of consecutive instances of the same obstacle to confirm it
#define THRESHOLD_GATE			2			//Count of consecutive instances of a gate to confirm it
#define OBSTACLE_CLEARING_DELAY	500			//[ms] Amount of time needed to complete the rotation
#define SENSOR_REFRESH_DELAY	100			//[ms] Minimum amount of time between new values dictated by the ToF sensor thread

static uint8_t last_type=0, count=0;
static uint16_t last_pos=0;


//Short function to move the robot straight
void set_speed(int speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}


//Short sign(x) function
int sign(float x){return (x > 0) - (x < 0);}

//Short function to rotate the robot
void rotate_lr(int lr)
{
	if(abs(lr)<MINIMUM_ROT_SPEED){lr =sign(lr)*MINIMUM_ROT_SPEED;}
	left_motor_set_speed(lr);
	right_motor_set_speed(-lr);
}

//Routine executed when the obstacle is of type UNKNOWN
void move_back (void)
{
	set_led(LED1, 1);
	set_speed(-SLOW_SPEED);
	while(VL53L0X_get_dist_mm()<TOO_CLOSE)
	{
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
	}
	set_speed(HALT);
	set_led(LED1, 0);
}


//Function to identify and confirm obstacles by looking at the data multiple times, the type of the obstacle is stored in last_type
bool recognize_obstacle(void)
{
	if (get_obstacle_type() && get_obstacle_type()!=UNKNOWN)
	{
		if (get_obstacle_type()==last_type && abs(last_pos-get_obstacle_pos()) < MAX_POS_SHIFT )
		{
			count++;
		}
		else{count=0;}

		last_type = get_obstacle_type();
		last_pos = get_obstacle_pos();

		if ((count>=THRESHOLD_EDGE_AND_GOAL || (count>=THRESHOLD_GATE && last_type == GATE)) && VL53L0X_get_dist_mm() < MAX_DIST_TO_CONSIDER)
		{
			return TRUE;
		}
	}
	return FALSE;
}

//Function to move forward and look for obstacles, returns the type of the obstacle if any has been found
uint8_t path_to_obstacle (void)
{
	systime_t start_time=chVTGetSystemTime();

	set_body_led(1);

	while(chVTGetSystemTime()<start_time + MAX_TRAVEL_TIME)
	{
		//Releases resources to the sensor handling threads to update
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);

		if (recognize_obstacle())
		{
			set_speed(HALT);
			set_body_led(0);
			return last_type;			//The type recognized was stored in the static variable last_type
		}

		if(VL53L0X_get_dist_mm()<SAFETY_DISTANCE)
		{
			set_speed(HALT);
			set_body_led(0);
			return UNKNOWN;
		}

		if(VL53L0X_get_dist_mm()<SLOW_DOWN_DISTANCE)
		{
			set_speed(SLOW_SPEED);
		}
		else
		{
			set_speed(FAST_SPEED);
		}
	}
	set_speed(HALT);
	set_body_led(0);
	return FALSE;
}

//Function to rotate toward the source and look for obstacles, returns the type of the obstacle if any has been found
uint8_t rotate_to_source (void)
{
	float turnangle = 0;
	uint8_t check_angle = 0;
	systime_t start_time = 0;
	reset_audio();
	chThdSleepMilliseconds(1000);

	while(!get_audio_status()){chThdSleepMilliseconds(500);}

	start_time=chVTGetSystemTime();

	while(1)
	{
		check_angle=0;
		turnangle = get_angle();
		rotate_lr(ROT_COEF*turnangle);
		chThdSleepMilliseconds(ROT_TIME);
		set_speed(HALT);
		if (chVTGetSystemTime()>start_time + MAX_ROT_TIME){break;}
		if (fabs(turnangle)>LARGE_ANGLE){reset_audio();  chThdSleepMilliseconds(1500);}
		chThdSleepMilliseconds(1000);

		for(uint8_t i = 0 ; i < STABILIZATION_TRIES; i++)
		{
			chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
			turnangle = get_angle();

			if (fabs(turnangle) < MAX_ANGLE_ERROR && get_audio_status()){check_angle++;}

			if(VL53L0X_get_dist_mm()<SAFETY_DISTANCE)
			{
				set_speed(HALT);
				set_body_led(0);
				return UNKNOWN;
			}
			if (recognize_obstacle()){return last_type;}
			if (check_angle>STABILIZED_AUDIO){ return FALSE;}
		}

	}
	return FALSE;
}


void move_around_edge(void)
{
	rotate_lr((get_obstacle_type()*2-3)*SLOW_SPEED);
	uint8_t type = get_obstacle_type();

	set_led(LED3, 1);
	set_led(LED7, 1);

	while (type == get_obstacle_type())
	{
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
	}
	chThdSleepMilliseconds(OBSTACLE_CLEARING_DELAY);
	set_speed(HALT);
	set_led(LED3, 0);
	set_led(LED7, 0);
}

void move_through_gate(void)
{
	set_speed(HALT);
	chThdSleepMilliseconds(1000);
	set_speed(RAM_SPEED);

	while(VL53L0X_get_dist_mm()>COLLISION_DISTANCE)
	{
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
	}

	chThdSleepMilliseconds(OBSTACLE_CLEARING_DELAY);

	set_speed(HALT);
}

void move_to_goal(void)
{
	rotate_lr(SLOW_SPEED);
	while(1)
	{
		set_led(LED1, 1);
		set_led(LED3, 1);
		set_led(LED5, 1);
		set_led(LED7, 1);
		chThdSleepMilliseconds(CELEBRATION_TIME);
		set_led(LED1, 0);
		set_led(LED3, 0);
		set_led(LED5, 0);
		set_led(LED7, 0);
		chThdSleepMilliseconds(CELEBRATION_TIME);

	}
}




