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

void set_speed(int speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}


void rotate_lr(int lr)
{
	left_motor_set_speed(lr*300);
	right_motor_set_speed(-lr*300);
}

void rotate_angle(float angle, uint16_t speed){
	int ang = ROT_COEF*fabs(angle);
	//chprintf((BaseSequentialStream *) &SD3, "(ANG_INT) = %d    \r\n", ang);
	if (ang)
	{
		if (angle > 0)
		{
			left_motor_set_speed(speed-300);
			right_motor_set_speed(speed+300);
			chThdSleepMilliseconds(ang);
			set_speed(speed);
		}
		else
		{
		left_motor_set_speed(speed+300);
		right_motor_set_speed(speed-300);
		chThdSleepMilliseconds(ang);
		set_speed(speed);
		}
	}
}

void move_back (void)
{
	set_led(LED1, 1);
	set_speed(-400);
	while(VL53L0X_get_dist_mm()<100)
	{
		chprintf((BaseSequentialStream *) &SD3, "(RETURNING) = %d    \r\n", VL53L0X_get_dist_mm());
		chThdSleepMilliseconds(200);
	}
	set_speed(0);
	rotate_angle(40.0f,0);
	set_led(LED1, 0);
}


uint8_t path_to_obstacle (uint8_t*index)
{
	uint8_t last_type = 0, count=0;
	uint16_t last_pos=0, start_distance=VL53L0X_get_dist_mm();

	while(start_distance-VL53L0X_get_dist_mm()<TRAVEL_DISTANCE)
	{
		if(VL53L0X_get_dist_mm()<SAFETY_DISTANCE)
		{
			move_back();
		}

		if(VL53L0X_get_dist_mm()<150)
		{
			set_speed(200);
		}
		else
		{
			set_speed(400);
		}

		for(uint8_t i = 0 ; i < MAXLINES; i++)
		{
			if (get_obstacle_type(i) && get_obstacle_type(i)!=UNKNOWN)
			{
				if (get_obstacle_type(i)==last_type && abs(last_pos-get_obstacle_pos(i)) < 50)
				{
					count++;
				}
				else{count=0;}

				last_type = get_obstacle_type(i);
				last_pos = get_obstacle_pos(i);

				//chprintf((BaseSequentialStream *) &SD3, "(COUNT): %d  \r\n", count);


				if (count>3 || (count>1 && (last_type == GATE || last_type == GOAL)))
				{
					//chprintf((BaseSequentialStream *) &SD3, "\n FOUND EDGE  \r\n", count);
					while(VL53L0X_get_dist_mm()>100)
					{
						//chprintf((BaseSequentialStream *) &SD3, "(distance): %d  \r\n", VL53L0X_get_dist_mm());
						chThdSleepMilliseconds(100);
					}
					set_speed(0);
					chThdSleepMilliseconds(500);
					*index = i;
					return TRUE;
				}
				break;
			}
		}

	}
	set_speed(0);
	chThdSleepMilliseconds(500);
	return FALSE;
}


void rotate_to_source (void)
{
	float turnangle = 0;
	float check_angle = 0;

	while(1)
			{
				turnangle = get_angle();
				rotate_angle(-turnangle, 0);
				chThdSleepMilliseconds(1000);

				if (fabs(turnangle) < 15.0f && get_audio_status()){check_angle++;}
				else { check_angle = 0; }

				if (check_angle>2){check_angle=0; break;}
			}
}


void move_around_edge(uint8_t index)
{
	rotate_lr(get_obstacle_type(index)*2-3);
	uint8_t stop = 0, type = get_obstacle_type(index);

	while (!stop)
	{
		chThdSleepMilliseconds(100);
		for(uint8_t i = 0 ; i < MAXLINES ; i++)
		{
			chThdSleepMilliseconds(50);
			if (get_obstacle_type(i)==type) {stop=0; break;}
			stop=1;
		}
	}
	chThdSleepMilliseconds(400);
	set_speed(0);
}

void move_through_gate(uint8_t index)
{
	rotate_lr(sign(IMAGE_BUFFER_SIZE/2-get_obstacle_pos(index))*0.25);
	while(abs(IMAGE_BUFFER_SIZE/2-get_obstacle_pos(index))<10)
	{
		chThdSleepMilliseconds(100);
	}
	set_speed(0);
	chThdSleepMilliseconds(1000);
	set_speed(1500);

	while(VL53L0X_get_dist_mm()>5)
	{
		chThdSleepMilliseconds(100);
	}

	set_speed(0);
}

void move_to_goal(uint8_t index)
{
	rotate_lr(2);
	chThdSleepMilliseconds(5000);
	set_speed(0);
}




