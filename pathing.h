/*
 * pathing.h
 *
 *  Created on: 3 May 2020
 *      Author: nicol
 */

#ifndef PATHING_H_
#define PATHING_H_



uint8_t path_to_obstacle(void);
uint8_t rotate_to_source (void);
void move_around_edge(void);
void move_through_gate(void);
void move_to_goal(void);
void move_back(void);




#endif /* PATHING_H_ */
