#ifndef AUDIO_H_
#define AUDIO_H_

extern void audio_init(void);
extern void audio_set_frequency(int freqHz);
extern void audio_generate_tone(int freqHz, int msOn, int msOff = 0, int numBeeps = 1);
extern void audio_toggle_mute();

#endif