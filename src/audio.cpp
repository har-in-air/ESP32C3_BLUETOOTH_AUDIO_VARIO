#include <Arduino.h>
#include "config.h"
#include "audio.h"

static bool IsMuted;
static const uint8_t channel = 0;

static void audio_on() {
	digitalWrite(pinAudioEn, LOW);
	ledcWrite(channel, 2048);
	ledcAttachPin(pinAudio, channel);
}

static void audio_off() {
	digitalWrite(pinAudioEn, HIGH);
	ledcWrite(channel, 0);
	ledcDetachPin(pinAudio);
	pinMode(pinAudio, OUTPUT);
	digitalWrite(pinAudio, 0);
}

void audio_toggle_mute() {
	IsMuted = !IsMuted;
	if( IsMuted )
		audio_off();
	else
		audio_on();
}

void audio_init(void) {
	pinMode(pinAudioEn, OUTPUT_OPEN_DRAIN); // output enable for 74HC240, active low
	IsMuted = false;
	audio_toggle_mute();
}

void audio_generate_tone(int freqHz, int msOn, int msOff, int numBeeps) {
	while(numBeeps--) {
		audio_on();
		ledcWriteTone(channel, freqHz);
		delay(msOn);
		audio_off();
		if( numBeeps )
			delay(msOff);
	}
}

void audio_set_frequency(int freqHz) {
	if (!IsMuted) {
		if (freqHz > 0) {
			digitalWrite(pinAudioEn, LOW);
			ledcWriteTone(channel, freqHz);	
		}
		else {
			digitalWrite(pinAudioEn, HIGH);
			ledcWriteTone(channel, 1);	
		}
	}
}
