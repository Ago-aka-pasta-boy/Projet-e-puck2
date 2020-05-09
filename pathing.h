/*
 * pathing.h
 *
 *  Created on: 3 May 2020
 *      Author: nicol
 */

#ifndef PATHING_H_
#define PATHING_H_

#define ROT_COEF 				7
#define SAFETY_DISTANCE 		40
#define TRAVEL_TIME				3000
#define ROT_TIME 				5000
#define COLLISION_DISTANCE 		30
#define MINIMUM_ROT_SPEED		175		//turning at a lower speed starts to make the robot shake instead of turning properly
#define SLOW_DOWN_DISTANCE		200
#define SLOW_SPEED				300
#define FAST_SPEED				500
#define HALT					0
#define	MAX_FILTER_CONVERGENCE	40
#define STABILIZED_AUDIO		5
#define MAX_ANGLE_ERROR			15.0f
#define CHARGING_SPEED			1000
#define CELEBRATION_TIME		5000
#define SENSOR_REFRESH_DELAY	100		//in ms, used inside a sleep()
#define TOO_CLOSE				80		//in mm, the distance from the obstacle that the robot must reach when moving back
#define MAX_POS_SHIFT			50
#define MAX_DIST_TO_CONSIDER	100
#define THRESHOLD_EDGE_AND_GOAL 3
#define THRESHOLD_GATE			1


uint8_t path_to_obstacle(void);
uint8_t rotate_to_source (void);
void move_around_edge(void);
void move_through_gate(void);
void move_to_goal(void);




#endif /* PATHING_H_ */
