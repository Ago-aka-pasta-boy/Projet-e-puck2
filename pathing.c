/*

File    : pathing.c
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Pathing file implementing the different pathing and movement functions as well as interpreting information from the audio and image processing modules
*/

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
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
#define STABILIZED_AUDIO		3			//The angle will be considered stable if 3 tries are accepted
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
#define AUDIO_SETTLING_TIME		1000		//[ms] Thread sleep time needed to allow audio values to stabilize
#define LARGE_ANGLE_SETTLING	1500		//[ms] Extra thread sleep time needed to allow audio values to stabilize after a large rotation

static uint8_t last_type=0, count=0;
static uint16_t last_pos=0;

//Short sign(x) function
int sign(float x){return (x > 0) - (x < 0);}


//Short function to move the robot straight
void set_speed(int speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}


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
		//Checks if the obstacle type and position are consistent
		if (get_obstacle_type()==last_type && abs(last_pos-get_obstacle_pos()) < MAX_POS_SHIFT )
		{
			count++;
		}
		else{count=0;}

		last_type = get_obstacle_type();
		last_pos = get_obstacle_pos();

		//Depending on type, a different number of confirmations is needed, as well as a certain proximity to the obstacle
		if ((count>=THRESHOLD_EDGE_AND_GOAL || (count>=THRESHOLD_GATE && last_type == GATE)) && VL53L0X_get_dist_mm() < MAX_DIST_TO_CONSIDER)
		{
			return TRUE;
		}
	}
	//Return FALSE if no obstacle was confirmed
	return FALSE;
}


//Function to path forward and look for obstacles, returns the type of the obstacle if any are confirmed (FALSE if none)
uint8_t path_to_obstacle (void)
{
	systime_t start_time=chVTGetSystemTime();

	set_body_led(1);

	//Breaks if MAX_TRAVEL_TIME has elapsed to return to localizing the audio source
	while(chVTGetSystemTime()<start_time + MAX_TRAVEL_TIME)
	{
		//Releases resources to the sensor handling threads to update
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);

		//Recognize_obstacle returns TRUE if obstacle is found, type stored in static variable last_type
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


//Function to rotate towards the source and look for obstacles, returns the type of the obstacle if any has been found (stored in last_type)
uint8_t rotate_to_source (void)
{
	float turnangle = 0;
	uint8_t check_angle = 0;
	systime_t start_time = 0;
	reset_audio();

	//Allows the audio values to settle
	chThdSleepMilliseconds(AUDIO_SETTLING_TIME);





	//Waits for the frequency to be picked up
	while(!get_audio_status()){chThdSleepMilliseconds(500);}

	start_time=chVTGetSystemTime();

	while(1)
	{
		check_angle=0;
		turnangle = get_angle();

		//Rotates at a speed proportional to the angle received from audio processing
		rotate_lr(ROT_COEF*turnangle);
		chThdSleepMilliseconds(ROT_TIME);
		set_speed(HALT);

		//Limits the time spent looking for the audio source
		if (chVTGetSystemTime()>start_time + MAX_ROT_TIME){break;}

		//If the angle turned was large, reset the audio values and allow more time to settle
		if (fabs(turnangle)>LARGE_ANGLE){reset_audio();  chThdSleepMilliseconds(LARGE_ANGLE_SETTLING);}

		//Allows the audio values to settle
		chThdSleepMilliseconds(AUDIO_SETTLING_TIME);

		//Polls the angle data STABILIZATION_TRIES times while allowing sensor refresh and looking for obstacles
		for(uint8_t i = 0 ; i < STABILIZATION_TRIES; i++)
		{
			chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
			turnangle = get_angle();

			//Counts polled angles around 0
			if (fabs(turnangle) < MAX_ANGLE_ERROR && get_audio_status()){check_angle++;}

			//Looks for obstacles
			if (recognize_obstacle()){return last_type;}

			//Return UNKNOWN if close unidentified obstacle is detected
			if(VL53L0X_get_dist_mm()<SAFETY_DISTANCE)
			{
				set_speed(HALT);
				set_body_led(0);
				return UNKNOWN;
			}

			//If count is high enough, we return to moving forward (No obstacle = FALSE)
			if (check_angle>=STABILIZED_AUDIO){ return FALSE;}
		}

	}
	//If MAX_ROT_TIME has elapsed, we continue forward with a FALSE = no obstacle
	return FALSE;
}


//Function to rotate around a recognized edge
void move_around_edge(void)
{
	//Rotates in direction dependent on edge type
	if (last_type==LEFT_EDGE){rotate_lr(-SLOW_SPEED);}
	else{rotate_lr(SLOW_SPEED);}

	uint8_t type = last_type;

	set_led(LED3, 1);
	set_led(LED7, 1);

	//Turns until the obstacle is out of the cameras FOV while allowing sensor refresh
	while (type == get_obstacle_type())
	{
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
	}

	//Delay to turn and clear the obstacle
	chThdSleepMilliseconds(OBSTACLE_CLEARING_DELAY);
	set_speed(HALT);
	set_led(LED3, 0);
	set_led(LED7, 0);
}


//Function to ram recognized gates
void ram_gate(void)
{
	//Waits for suspense, then charges at high speed
	set_speed(HALT);
	chThdSleepMilliseconds(1000);
	set_speed(RAM_SPEED);

	//Waits for a small distance to be registered, indicating collision
	while(VL53L0X_get_dist_mm()>COLLISION_DISTANCE)
	{
		chThdSleepMilliseconds(SENSOR_REFRESH_DELAY);
	}

	//Coasts to clear obstacle
	chThdSleepMilliseconds(OBSTACLE_CLEARING_DELAY);

	set_speed(HALT);
}


//Function to celebrate the completion of the audio localization
void goal_success(void)
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
