/*

File    : communications.h

Code given in the EPFL MICRO-315 TP (Spring Semester 2020)
*/

#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);


#endif /* COMMUNICATIONS_H */
