/*

File    : audio_processing.c
Author  : Nicolas Zaugg, Sylvain Pellegrini
Date    : 10 may 2020

Audio processing functions to handle the data from the mp45dt02 MEMS microphones

Adapted from the code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include "leds.h"
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//Defines
//#define FILTER_SIZE		20 //Size of the moving average filter's buffer
#define A				0.925f
#define B				0.075f

#define MIN_VALUE_THRESHOLD	10000 //Threshold value for the max_frequency function

#define MIN_FREQ		59	//we don't analyze before this index to not use resources for nothing
#define FREQ_SENDER		64 //32=488HZ 64=876 ?!? 976 bro :P
#define MAX_FREQ		69	//we don't analyze after this index to not use resources for nothing
#define MAX_ERROR		1	//frequency tolerance


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);


//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
//Arrays containing the phase differences from the audio data
static float phase_diff_lr;//[FILTER_SIZE] ={ 0};
static float phase_diff_fb;//[FILTER_SIZE] ={ 0};
//Current phase differences
//static float current_phase_diff_lr=0;
//static float current_phase_diff_fb=0;
//Angle to the sound source
static float angle = 0;
//Moving averages for the phase differences
static float mov_avg_lr = 0;
static float mov_avg_fb = 0;
//Audio status variable
static uint8_t audio_status = NO_AUDIO;


//Simple function used to detect the highest value in a buffer
uint16_t max_frequency(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	return max_norm_index;
}


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples)
{
	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE))
		{
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//Signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		//Finds the frequency of each microphone
		uint16_t freq_left = max_frequency(micLeft_output);
		uint16_t freq_right = max_frequency(micRight_output);
		uint16_t freq_front = max_frequency(micFront_output);
		uint16_t freq_back = max_frequency(micBack_output);


		//CHECK ALL ARE SAME

		//Introduce the microphone data directly into the motor functions (with cooefs)

		//0.9*old + 0.1*new = new_filtered

		//detection of the wanted frequency and check if all microphones have the same max frequency
		if((abs(freq_left - FREQ_SENDER)<= MAX_ERROR) && (freq_right == freq_left)&&
				(freq_front == freq_left)&&	(freq_back == freq_left))
		{
			//changing the audio status to the case detected
			audio_status = FREQ_1;
			//computing the phases from the data of two mics and their difference
			phase_diff_lr = atan2f(micLeft_cmplx_input[freq_left+1],micLeft_cmplx_input[freq_left])
												-atan2f(micRight_cmplx_input[freq_right+1],micRight_cmplx_input[freq_right]);
			phase_diff_fb = atan2f(micFront_cmplx_input[freq_front+1],micFront_cmplx_input[freq_front])
												-atan2f(micBack_cmplx_input[freq_back+1],micBack_cmplx_input[freq_back]);

			//moving average that only considers left and right microphone phase differences smaller than 1 to reject some noise
			if(fabs(phase_diff_lr)<1)
			{
//				//all the old values in the buffer are pulled to the left
//				for (uint8_t i = 0; i < FILTER_SIZE-1; i++)
//				{
//					phase_diff_lr[i]=phase_diff_lr[i+1];
//				}
//
//				//the new value is put in the last index of the buffer
//				phase_diff_lr[FILTER_SIZE-1] = current_phase_diff_lr;
//
//				//here an average with linear weights, so the last new value is the most important
//				for (uint8_t i = 0; i < FILTER_SIZE; i++)
//				{
//					mov_avg_lr+=phase_diff_lr[i];//*i;
//				}
//				mov_avg_lr /= FILTER_SIZE;//*(FILTER_SIZE+1)/2;
				mov_avg_lr = A * mov_avg_lr + B * phase_diff_lr;


			}

			//here the same moving average principle is applied to phase differences between front and back
			if(fabs(phase_diff_fb)<1)
			{
//				for (uint8_t i = 0; i < FILTER_SIZE-1; i++)
//				{
//					phase_diff_fb[i]=phase_diff_fb[i+1];
//				}
//
//				//phase differences between microphones
//				phase_diff_fb[FILTER_SIZE-1] = current_phase_diff_fb;
//
//				for (uint8_t i = 0; i < FILTER_SIZE; i++)
//				{
//					mov_avg_fb+=phase_diff_fb[i];//*i;
//				}
//				mov_avg_fb /= FILTER_SIZE;//*(FILTER_SIZE+1)/2;
				mov_avg_fb = A * mov_avg_fb + B * phase_diff_fb;
			}

		}

		else
		{
			audio_status = NO_AUDIO;

		}
	}
}


//Resets the moving average
void reset_audio (void)
{
	mov_avg_lr = 0;
	mov_avg_fb = -0.5;
}


//Returns the current audio status
uint8_t get_audio_status(void)
{
	return audio_status;
}


//Returns the current angle
float get_angle(void)
{
	//Checks if the frequency is registered
	if (audio_status == NO_AUDIO){return 0;}
	//Angle calculation from the averaged and filtered phase differences. Mov_avg_fb is inverted to correct for the orientation.
	angle=atan2f(mov_avg_lr,-(mov_avg_fb))*360.0f/(2.0f*PI);

	return angle;
}


//Returns the LR moving average - mostly for testing
float get_lr(void)
{
	if (audio_status == NO_AUDIO){return 0;}
	return mov_avg_lr;
}


//Returns the FB moving average - mostly for testing
float get_fb(void)
{
	if (audio_status == NO_AUDIO){return 0;}
	return mov_avg_fb;
}


//Signals to send the result to the computer
void wait_send_to_computer(void)
{
	chBSemWait(&sendToComputer_sem);
}


//Returns the pointers for the audio buffers
float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
