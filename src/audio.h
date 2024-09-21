#ifndef AUDIO_H_
#define AUDIO_H_

extern bool IsMuted;

extern void audio_init(void);
extern void audio_set_frequency(int freqHz);
extern void audio_generate_tone(int freqHz, int ms);
extern void audio_beep(int freqHz, int onMs, int offMs, int numBeeps);
extern void audio_off();

#endif