#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//defs for TP_4 CAMREG
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
//-------------

#define MAXLINES				5


//Obstacle types
#define LEFT_EDGE				1
#define RIGHT_EDGE				2
#define GATE					3
#define GOAL					4
#define UNKNOWN					5


struct line {
	bool exist;
	uint16_t start;
	uint16_t end;
	uint16_t pos;
	uint16_t meanval;
};


struct obstacle{
	uint8_t type;
	uint16_t pos;
};

void extract_lines(uint8_t *buffer);
void extract_edges(uint8_t *buffer);
void extract_gate(void);
void extract_goal(void);

void clear_obstacle(uint8_t i);

uint8_t get_obstacle_type(void);

uint16_t get_obstacle_pos(void);


float get_distance_cm(void);

uint16_t get_line_position(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
