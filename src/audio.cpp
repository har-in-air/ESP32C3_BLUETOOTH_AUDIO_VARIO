#include <Arduino.h>
#include "config.h"
#include "audio.h"

static void audio_tone(int pin, int freqHz, int milliseconds);
static void audio_tone_off(int pin);

static void audio_tone(int pin, int freqHz, int milliseconds) {
	uint8_t channel = 0;
	digitalWrite(pinAudioEn, 0);
	ledcAttachPin(pin, channel);
	ledcWriteTone(channel, freqHz);
	delay(milliseconds);
	audio_tone_off(pin);
	}


static void audio_tone_off(int pin) {
	uint8_t channel = 0;
	ledcWrite(channel, 0);
	ledcDetachPin(pin);
	pinMode(pin, OUTPUT);
	digitalWrite(pin, 0);
	digitalWrite(pinAudioEn, 1);
	}


void audio_beep(int freqHz, int onMs, int offMs, int numBeeps) {
	while(numBeeps--) {
		audio_tone(pinAudio,freqHz, onMs);
		delay(offMs);
		}
	}


void audio_set_frequency(int freqHz) {
	uint8_t channel = 0;
	ledcAttachPin(pinAudio, channel);
	if (freqHz > 0) {
		digitalWrite(pinAudioEn, 0);
    	ledcWriteTone(channel, freqHz);	
		}
	else {
		digitalWrite(pinAudioEn, 1);
		ledcWrite(channel, 0);
		ledcDetachPin(pinAudio);
		pinMode(pinAudio, OUTPUT);
		digitalWrite(pinAudio, 0);
		}
	}


void audio_generate_tone(int freqHz, int ms) {
	digitalWrite(pinAudioEn, 0);
	audio_tone(pinAudio, freqHz, ms);
	digitalWrite(pinAudioEn, 1);
	}
