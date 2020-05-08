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

#include <sensors/proximity.h>
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
	if(abs(lr)<175){lr =sign(lr)*175;}
	if(abs(lr)>1500){lr =sign(lr)*1500;}
	left_motor_set_speed(lr);
	right_motor_set_speed(-lr);
}


void move_back (void)
{
	set_led(LED1, 1);
	set_speed(-400);
	while(VL53L0X_get_dist_mm()<80) //&& get_prox(1) > 180 && get_prox(2) > 180 && get_prox(7) > 180 && get_prox(8) > 180)
	{
		chThdSleepMilliseconds(100);
	}
	set_speed(0);
	set_led(LED1, 0);
}


uint8_t path_to_obstacle (void)
{
	uint8_t last_type = 0, count=0, current_obstacle = 0;
	uint16_t last_pos=0, start_distance=VL53L0X_get_dist_mm();
	systime_t start_time=chVTGetSystemTime();

	set_body_led(1);

	while(chVTGetSystemTime()<start_time + TRAVEL_TIME)
	{
			if (get_obstacle_type() && get_obstacle_type()!=UNKNOWN)
			{
				if (get_obstacle_type()==last_type && abs(last_pos-get_obstacle_pos()) < 50 && VL53L0X_get_dist_mm() < 100)
				{
					count++;
				}
				else{count=0;}

				last_type = get_obstacle_type();
				last_pos = get_obstacle_pos();

				//chprintf((BaseSequentialStream *) &SD3, "(COUNT): %d  \r\n", count);


				if (count>3 || (count>1 && (last_type == GATE || last_type == GOAL)))
				{
					//chprintf((BaseSequentialStream *) &SD3, "\n FOUND EDGE  \r\n", count);

					set_speed(0);
					set_body_led(0);
					return TRUE;
				}
			}




		if(VL53L0X_get_dist_mm()<SAFETY_DISTANCE) //|| get_prox(1) > 200 || get_prox(2) > 200 || get_prox(7) > 200 || get_prox(8) > 200)
		{
			set_body_led(0);
			move_back();
			return FALSE;
		}

		if(VL53L0X_get_dist_mm()<200)
		{
			set_speed(300);
		}
		else
		{
			set_speed(600);
		}
	}
	set_speed(0);
	set_body_led(0);
	return FALSE;
}


void rotate_to_source (void)
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
		//chprintf((BaseSequentialStream *) &SD3, "ANGLE %f  \r\n", turnangle);
		rotate_lr(ROT_COEF*turnangle);
		chThdSleepMilliseconds(500);
		set_speed(0);
		if (fabs(turnangle)>40){reset_audio();  chThdSleepMilliseconds(1500);}
		chThdSleepMilliseconds(1000);

		for(uint8_t i = 0 ; i < 5; i++)
		{
			chThdSleepMilliseconds(100);
			turnangle = get_angle();

			if (fabs(turnangle) < 15.0f && get_audio_status()){check_angle++;}

			if (check_angle>5){ return;}
		}

	}
}


void move_around_edge(void)
{
	rotate_lr((get_obstacle_type()*2-3)*300);
	uint8_t stop = 0, type = get_obstacle_type();

	set_led(LED3, 1);
	set_led(LED7, 1);

	while (type == get_obstacle_type())
	{
		chThdSleepMilliseconds(100);
	}
	chThdSleepMilliseconds(500);
	set_speed(0);
	set_led(LED3, 0);
	set_led(LED7, 0);
}

void move_through_gate(void)
{
	rotate_lr(sign(IMAGE_BUFFER_SIZE/2-get_obstacle_pos())*100);
	while(abs(IMAGE_BUFFER_SIZE/2-get_obstacle_pos())<10)
	{
		chThdSleepMilliseconds(100);
	}
	set_speed(0);
	chThdSleepMilliseconds(1000);
	set_speed(1000);

	while(VL53L0X_get_dist_mm()>COLLISION_DISTANCE)
	{
		chThdSleepMilliseconds(100);
	}

	chThdSleepMilliseconds(1000);

	set_speed(0);
}

void move_to_goal(void)
{
	rotate_lr(300);
	chThdSleepMilliseconds(5000);
	set_speed(0);
}




