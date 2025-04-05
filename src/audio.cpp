#include <Arduino.h>
#include "config.h"
#include "audio.h"

#define DUTY_50_PCNT 2048
#define DRIVER_ENABLE 0
#define DRIVER_DISABLE 1

bool IsMuted = false;

void audio_generate_tone(int freqHz, int milliseconds) {
	ledcSetClockSource(LEDC_AUTO_CLK);
	ledcAttach((uint8_t)pinAudio, (uint32_t)freqHz, 12);
	ledcWrite((uint8_t)pinAudio, DUTY_50_PCNT);
	digitalWrite(pinAudioEn, DRIVER_ENABLE);
	delay(milliseconds);
	audio_off();
	}


void audio_off() {
	digitalWrite(pinAudioEn, DRIVER_DISABLE);
	ledcWrite((uint8_t)pinAudio, 0);
	ledcDetach((uint8_t)pinAudio);
	pinMode(pinAudio, OUTPUT);
	digitalWrite(pinAudio, 0);
	}


void audio_beep(int freqHz, int onMs, int offMs, int numBeeps) {
	while(numBeeps--) {
		audio_generate_tone(freqHz, onMs);
		delay(offMs);
		}
	}


void audio_set_frequency(int freqHz) {
	if (!IsMuted) {
		if (freqHz > 0) {
			ledcSetClockSource(LEDC_AUTO_CLK);
			ledcAttach((uint8_t)pinAudio, (uint32_t)freqHz, 12);
			ledcWrite((uint8_t)pinAudio, DUTY_50_PCNT);
			digitalWrite(pinAudioEn, DRIVER_ENABLE);
			}
		else {
			audio_off();
			}
		}
	}


