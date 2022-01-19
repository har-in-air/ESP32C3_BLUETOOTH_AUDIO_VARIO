#ifndef AUDIO_H_
#define AUDIO_H_

void audio_set_frequency(int freqHz);
void audio_generate_tone(int freqHz, int ms);
void audio_beep(int freqHz, int onMs, int offMs, int numBeeps);

#endif