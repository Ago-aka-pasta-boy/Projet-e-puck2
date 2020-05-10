/*

File    : audio_processing.h
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Audio processing functions to handle the data from the mp45dt02 MEMS microphones

Adapted from the code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


//Possible Audio states
#define NO_AUDIO			0
#define AUDIO_DETECTED		1


//Resets the moving average to speed up the settling time after a large rotation
void reset_audio (void);

//Callback for the audio processing
void processAudioData(int16_t *data, uint16_t num_samples);

//Returns the current audio status
uint8_t get_audio_status(void);

//Returns the current angle
float get_angle(void);


#endif /* AUDIO_PROCESSING_H */
