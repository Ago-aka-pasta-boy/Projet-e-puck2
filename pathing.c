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

static uint8_t last_type=0, count=0;
static uint16_t last_pos=0;

void set_speed(int speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}


//Short sign(x) function
int sign(float x){return (x > 0) - (x < 0);}

void rotate_lr(int lr)
{
	if(abs(lr)<MINIMUM_ROT_SPEED){lr =sign(lr)*MINIMUM_ROT_SPEED;}
	left_motor_set_speed(lr);
	right_motor_set_speed(-lr);
}


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


uint8_t recognize_obstacle(void)
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

		if ((count>THRESHOLD_EDGE_AND_GOAL || (count>THRESHOLD_GATE && last_type == GATE)) && VL53L0X_get_dist_mm() < MAX_DIST_TO_CONSIDER)
		{
			return last_type;
		}
	}
	return FALSE;
}

uint8_t path_to_obstacle (void)
{
	systime_t start_time=chVTGetSystemTime();

	set_body_led(1);

	while(chVTGetSystemTime()<start_time + TRAVEL_TIME)
	{
		//Releases resources to the sensor handling threads to update
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);

		if (recognize_obstacle())
		{
			set_speed(HALT);
			set_body_led(0);
			return last_type;
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


uint8_t rotate_to_source (void)
{
	float turnangle = 0;
	uint8_t check_angle = 0;
	systime_t start_time = 0;
	reset_audio();
	chThdSleepMilliseconds(1000);

	while(!get_audio_status()){chThdSleepMilliseconds(500);}

	start_time=chVTGetSystemTime();

	while(chVTGetSystemTime()<start_time + ROT_TIME)
	{
		check_angle=0;
		turnangle = get_angle();
		rotate_lr(ROT_COEF*turnangle);
		chThdSleepMilliseconds(500);
		set_speed(HALT);
		if (fabs(turnangle)>LARGE_ANGLE){reset_audio();  chThdSleepMilliseconds(1500);}
		chThdSleepMilliseconds(1000);

		for(uint8_t i = 0 ; i < STABILIZED_AUDIO; i++)
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




