#include <Arduino.h>
#include "config.h"
#include "adc.h"

#define NUM_AVG_SAMPLES 4

int adc_sample_average() {
	int adcSample = 0;
		for (int inx = 0; inx < NUM_AVG_SAMPLES; inx++) {
		adcSample += analogRead(A0);
		delay(1);
		}
	adcSample /= NUM_AVG_SAMPLES;
	return adcSample;  
	}


// voltage divider with 12K and 3.3K to scale 4.2V down to < 1.0V for the ESP32-C3 ADC
// two-point calibration for slope & intercept
#define V1  3.286f
#define ADC1  765.0f
#define V2  4.050f
#define ADC2  932.0f

float adc_battery_voltage(int sample) {
	float slope = (V2 - V1)/(ADC2 - ADC1);
	float voltage = slope*(sample - ADC1) + V1;
	return voltage;
	}
