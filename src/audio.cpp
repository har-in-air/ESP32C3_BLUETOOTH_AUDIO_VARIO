#include <Arduino.h>
#include "config.h"
#include "audio.h"

static bool IsMuted;
static const uint8_t channel = 0;

static void audio_on() {
	digitalWrite(pinAudioEn, LOW);
	ledcWrite(channel, 0x1FF);
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
	dbg_printf(("Audio is now %smuted\n", IsMuted ? "" : "NOT "));
	if( IsMuted )
		audio_off();
	else
		audio_on();
}

void audio_init(void) {
	pinMode(pinAudioEn, OUTPUT_OPEN_DRAIN); // output enable for 74HC240, active low
	ledcSetup(channel, 1000, 14);
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
		// dbg_printf(("Freq to hear: %d\n", freqHz));
		if (freqHz > 0) {
			audio_on();
			ledcWriteTone(channel, freqHz);	
		}
		else {
			audio_off();
		}
	}
}
