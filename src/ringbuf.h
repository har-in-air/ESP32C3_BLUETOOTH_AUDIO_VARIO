#ifndef RINGBUF_H_
#define RINGBUF_H_

#define RINGBUF_SIZE    20

typedef struct RINGBUF_ {
   int head;
   float buffer[RINGBUF_SIZE];
} RINGBUF;

void ringbuf_init();
void ringbuf_add_sample(float sample);
float ringbuf_average_oldest_samples(int numSamples);
float ringbuf_average_newest_samples(int numSamples);

#endif
