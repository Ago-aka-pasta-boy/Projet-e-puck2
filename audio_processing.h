/*

File    : audio_processing.h
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Audio processing functions to handle the data from the mp45dt02 MEMS microphones

Adapted from the code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

//Size of the buffers
#define FFT_SIZE 	1024

//Possible Audio states
#define NO_AUDIO	0
#define FREQ_1		1

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;


//NEW
void reset_audio (void);

//Main audio processing function
void processAudioData(int16_t *data, uint16_t num_samples);

//Returns the current audio status
uint8_t get_audio_status(void);

//Returns the current angle
float get_angle(void);


#endif /* AUDIO_PROCESSING_H */
