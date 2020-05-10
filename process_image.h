#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//defs for TP_4 CAMREG
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
//-------------

#define MAX_LINES				5
#define STATIC_NOISE			100
#define	RATIO_TO_QUALIFY_LINES	0.7
#define RATIO_TO_QUALIFY_EDGES	2
#define RATIO_TO_CONFIRM_EDGES	0.3
#define MIN_LINES_FOR_GOAL		2

//Obstacle types

#define LEFT_EDGE				1
#define RIGHT_EDGE				2
#define GATE					3
#define GOAL					4
#define UNKNOWN					5


uint8_t get_obstacle_type(void);
uint16_t get_obstacle_pos(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
