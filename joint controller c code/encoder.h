#ifndef ENCODER__H__
#define ENCODER__H__

void encoder_init();          // initialize the encoder module

int encoder_ticks();          // read the encoder, in ticks

float encoder_angle();          // read the encoder angle in 1/10 degrees

void encoder_reset();         // reset the encoder position

#endif

