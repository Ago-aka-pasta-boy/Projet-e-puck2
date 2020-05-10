/*

File    : pathing.h
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Pathing file implementing the different pathing and movement functions as well as interpreting information from the audio and image processing modules
*/


#ifndef PATHING_H_
#define PATHING_H_

//Routine executed when the obstacle is of type UNKNOWN
void move_back(void);

//Function to path forward and look for obstacles, returns the type of the obstacle if any are confirmed (FALSE if none)
uint8_t path_to_obstacle (void);

//Function to rotate towards the source and look for obstacles, returns the type of the obstacle if any has been found (stored in last_type)
uint8_t rotate_to_source (void);

//Function to rotate around a recognized edge
void move_around_edge(void);

//Function to ram recognized gates
void ram_gate(void);

//Function to celebrate the completion of the audio localization
void goal_success(void);

#endif /* PATHING_H_ */
