#include <Arduino.h>
#include "config.h"
#include "audio.h"

bool IsMuted = false;
static const uint8_t channel = 0;

void audio_init(void) {
	pinMode(pinAudioEn, OUTPUT_OPEN_DRAIN); // output enable for 74HC240, active low
	digitalWrite(pinAudioEn, HIGH);
	ledcWrite(channel, 2048);
	ledcAttachPin(pinAudio, channel);
}

static void audio_tone(int freqHz, int milliseconds) {
	digitalWrite(pinAudioEn, 0);
	ledcWriteTone(channel, freqHz);
	delay(milliseconds);
	audio_off();
	}


void audio_off() {
	digitalWrite(pinAudioEn, 1);
	ledcWrite(channel, 0);
	pinMode(pinAudio, OUTPUT);
	digitalWrite(pinAudio, 0);
	}


void audio_beep(int freqHz, int onMs, int offMs, int numBeeps) {
	while(numBeeps--) {
		audio_tone(freqHz, onMs);
		delay(offMs);
		}
	}


void audio_set_frequency(int freqHz) {
	if (!IsMuted) {
		if (freqHz > 0) {
			digitalWrite(pinAudioEn, 0);
			ledcWriteTone(channel, freqHz);	
			}
		else {
			audio_off();
			}
		}
	}


void audio_generate_tone(int freqHz, int ms) {
	digitalWrite(pinAudioEn, 0);
	audio_tone(freqHz, ms);
	digitalWrite(pinAudioEn, 1);
	}
