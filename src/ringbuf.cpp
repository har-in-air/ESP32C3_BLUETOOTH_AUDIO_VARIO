#include <Arduino.h>
#include "ringbuf.h"

static const char* TAG = "ringbuf";

static RINGBUF RingBuf;

void ringbuf_init() {
   memset(RingBuf.buffer,0,RINGBUF_SIZE);
   RingBuf.head = RINGBUF_SIZE-1;
   }

void ringbuf_add_sample(float sample) {
   RingBuf.head++;
   if (RingBuf.head >= RINGBUF_SIZE) RingBuf.head = 0;
   RingBuf.buffer[RingBuf.head] = sample;
   }


float ringbuf_average_oldest_samples(int numSamples) {
   int index = RingBuf.head+1; // oldest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index >= RINGBUF_SIZE) index = 0;
      accum += RingBuf.buffer[index];
      index++;
      }
   return accum/numSamples;
   }   

float ringbuf_average_newest_samples(int numSamples) {
   int index = RingBuf.head; // newest Sample
   float accum = 0.0f;
   for (int count = 0; count < numSamples; count++) {
      if (index < 0) index = RINGBUF_SIZE - 1;
      accum += RingBuf.buffer[index];
      index--;
      }
   return accum/numSamples;
   }   
