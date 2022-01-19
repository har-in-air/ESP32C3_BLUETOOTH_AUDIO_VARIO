#include <Arduino.h>
#include "config.h"
#include "audio.h"

static void audio_tone_on(int pin, int freqHz, int milliseconds);
static void audio_tone_off(int pin);

static void audio_tone_on(int pin, int freqHz, int milliseconds) {
	uint8_t channel = 0;
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
	}


void audio_beep(int freqHz, int onMs, int offMs, int numBeeps) {
	while(numBeeps--) {
		audio_tone_on(pinAudio,freqHz, onMs);
		delay(offMs);
		}
	}


void audio_set_frequency(int freqHz) {
	if (freqHz > 0) {
		#if (CFG_L9110S == true)
		digitalWrite(pinL9110Pwr, 1);
		#endif	
		audio_tone_on(pinAudio, freqHz, 10000);
		}
	else {
		#if (CFG_L9110S == true)
		digitalWrite(pinL9110Pwr, 0);
		#endif	
		audio_tone_off(pinAudio);
		}
	}


void audio_generate_tone(int freqHz, int ms) {
	#if (CFG_L9110S == true)
	digitalWrite(pinL9110Pwr, 1);
	#endif	
	audio_tone_on(pinAudio, freqHz, ms);
	delay(ms);
	#if (CFG_L9110S == true)
	digitalWrite(pinL9110Pwr, 0);
	#endif	
	}
