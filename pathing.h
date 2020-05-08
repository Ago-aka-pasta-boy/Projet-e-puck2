/*
 * pathing.h
 *
 *  Created on: 3 May 2020
 *      Author: nicol
 */

#ifndef PATHING_H_
#define PATHING_H_

#define ROT_COEF 			7
#define SAFETY_DISTANCE 	40
#define TRAVEL_TIME			3000
#define ROT_TIME 			5000
#define COLLISION_DISTANCE 	30

void set_speed(int speed);
void rotate_lr(int lr);
uint8_t path_to_obstacle(void);
void rotate_to_source (void);
void move_back (void);
void move_around_edge(void);
void move_through_gate(void);
void move_to_goal(void);




#endif /* PATHING_H_ */
