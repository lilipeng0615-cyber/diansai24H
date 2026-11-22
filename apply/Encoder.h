#ifndef  __ENCODER_H
#define  __ENCODER_H



#include "databyte.h"

void EncoderInit(Encoder_t *Encoder);
void EncodersInit(Motors_t *Motor);
void EncoderDataUpdate(Motors_t *Motor);
float EncoderTotalLengthGet(Motors_t *Motors);
float EncoderLengthDiffGet(Motors_t *Motors);
float EncoderWheelAngleDegGet(const Encoder_t *Encoder);
float EncoderTurnAngleGet(Motors_t *Motors);
float EncoderTurnAngleDegGet(Motors_t *Motors);
float EncoderTurnRateGet(Motors_t *Motors);
void EncoderLengthClear(Motors_t *Motors);



#endif