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

#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//TESTING
#include "leds.h"


//Defines
#define FFT_SIZE 			1024			//Size of the buffer where the result of the FFT is stored

#define MIN_VALUE_THRESHOLD	10000 			//Threshold value for the max_frequency function
#define MIN_FREQ			59				//We don't analyze before this index to not use resources for nothing
#define FREQ_SOURCE			64				//64 * 15.3 = 980hz, in reality best results were observed at 990hz
#define MAX_FREQ			69				//We don't analyze after this index to not use resources for nothing
#define MAX_ERROR			1				//Frequency tolerance

#define A					0.925f			//Coefficient for the low-pass filter to apply to the past values
#define B					0.075f			//= 1-A, coefficient for the new value
#define BASE_LR_VALUE		0				//Value of the phase difference between left and right microphones when the source is in front
#define BASE_FB_VALUE		-0.5f			//Value of the phase difference between front and back microphones when the source is in front


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

//Moving averages for the phase differences
static float mov_avg_lr = 0;
static float mov_avg_fb = 0;
//Audio status variable
static uint8_t audio_status = NO_AUDIO;


//Simple function used to detect the highest value in the buffer
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
	//Number of samples in the buffer that we have to fill to FFT_SIZE
	uint16_t nb_samples = 0;
	//Phase differences computed from the audio data between respectively left-right and front-back microphones
	float phase_diff_lr, phase_diff_fb;
	//Loop to fill the buffers
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
		//FFT processing
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		//Magnitude processing
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		nb_samples = 0;

		//Finds the frequency of each microphone
		uint16_t freq_left = max_frequency(micLeft_output);
		uint16_t freq_right = max_frequency(micRight_output);
		uint16_t freq_front = max_frequency(micFront_output);
		uint16_t freq_back = max_frequency(micBack_output);

		//Detection of the wanted frequency and check if all microphones have the same max frequency
		if((abs(freq_left - FREQ_SOURCE)<= MAX_ERROR) && (freq_right == freq_left)&&
				(freq_front == freq_left)&&	(freq_back == freq_left))
		{


			//Update the audio status
			audio_status = AUDIO_DETECTED;

			//Computing the phases from the data of two opposite mics and subtracting to obtain the phase difference
			phase_diff_lr = atan2f(micLeft_cmplx_input[freq_left+1],micLeft_cmplx_input[freq_left])
												-atan2f(micRight_cmplx_input[freq_right+1],micRight_cmplx_input[freq_right]);
			phase_diff_fb = atan2f(micFront_cmplx_input[freq_front+1],micFront_cmplx_input[freq_front])
												-atan2f(micBack_cmplx_input[freq_back+1],micBack_cmplx_input[freq_back]);

			//Average emulating a low-pass filter, considers only left and right microphone phase differences smaller than 1 to reject some noise
			if(fabs(phase_diff_lr)<1)
			{
				mov_avg_lr = A * mov_avg_lr + B * phase_diff_lr;
			}

			//Same principle is applied to the front back phase differences
			if(fabs(phase_diff_fb)<1)
			{
				mov_avg_fb = A * mov_avg_fb + B * phase_diff_fb;
			}
		}
		//If the frequencies do not match
		else
		{
			audio_status = NO_AUDIO;
		}
	}
}


//Resets the moving average to speed up the settling time after a large rotation
void reset_audio (void)
{
	mov_avg_lr = BASE_LR_VALUE;
	mov_avg_fb = BASE_FB_VALUE;
}


//Returns the current audio status
uint8_t get_audio_status(void)
{
	return audio_status;
}


//Returns the current angle
float get_angle(void)
{
	float angle =0;
	//Checks if the frequency is registered
	if (audio_status == NO_AUDIO){return 0;}
	//The angle to sound source is computed from the filtered phase differences. Mov_avg_fb is inverted to correct for the orientation.
	angle=atan2f(mov_avg_lr,-(mov_avg_fb))*360.0f/(2.0f*PI);
	return angle;
}

