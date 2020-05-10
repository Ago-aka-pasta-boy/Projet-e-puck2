/*

File    : process_image.h
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Image capturing and processing functions to handle the data from the po8030 camera

Adapted from the code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//Obstacle types
#define LEFT_EDGE				1
#define RIGHT_EDGE				2
#define GATE					3
#define GOAL					4
#define UNKNOWN					5

//Returns the type of the obstacle in the 0th index of the array, considered the currently seen obstacle, to external modules
uint8_t get_obstacle_type(void);

//Returns the position of the obstacle in the 0th index of the array, considered the currently seen obstacle, to external modules
uint16_t get_obstacle_pos(void);

//Starts the image capture and image processing threads
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
