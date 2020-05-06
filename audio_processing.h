#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define NO_AUDIO	0
#define FREQ_1		1
#define FREQ_2		2

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


void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

float get_angle(void); //returns the current angle to source

uint8_t get_audio_status(void);

float get_lr(void);

float get_fb(void);

void compute_angle(void); //computes the angle to source

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

int sign(float x);

#endif /* AUDIO_PROCESSING_H */
