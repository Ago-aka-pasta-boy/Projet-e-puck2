/*
 * pathing.h
 *
 *  Created on: 3 May 2020
 *      Author: nicol
 */

#ifndef PATHING_H_
#define PATHING_H_

#define ROT_COEF 			8.0f
#define SAFETY_DISTANCE 	40
#define TRAVEL_DISTANCE		150


void reset_motors(void);
void rotate_lr(int lr);
void rotate_angle(float angle, uint16_t speed);
uint8_t path_to_obstacle (uint8_t*index);
void rotate_to_source (void);
void move_around_edge(uint8_t index);
void move_through_gate(uint8_t index);
void move_to_goal(uint8_t index);




#endif /* PATHING_H_ */
