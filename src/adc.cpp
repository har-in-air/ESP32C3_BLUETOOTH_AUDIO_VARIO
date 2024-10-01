#include <Arduino.h>
#include <driver/adc.h>
#include "config.h"
#include "adc.h"

#define CHAN_VBAT    ADC1_CHANNEL_0 //((adc1_channel_t)0)
// two-point calibration for slope & intercept using ATTEN_DB_0, 10bit resolution
#define V1		3.252f
#define ADC1	723.0f
#define V2		4.080f
#define ADC2	908.0f

static const float battery_max = 4.20; //maximum voltage of battery
static const float battery_min = 3.0;  //minimum voltage of battery before shutdown

static float BatteryVoltage;

static void adc_update_average_battery_voltage(unsigned nr_of_samples) {
	float sum = 0.0f;
	for (int i = 0; i < nr_of_samples; i++) {
		adc_update_battery_voltage();
		sum += BatteryVoltage;
		delay(1);
	}
	BatteryVoltage = sum / nr_of_samples;  
}

void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12); // no 10-bit resolution option !?
	// need to ensure maximum voltage at adc pin is < 800mV for ATTEN_DB_0
    adc1_config_channel_atten(CHAN_VBAT, ADC_ATTEN_DB_0);
	adc_update_average_battery_voltage(4);
}

void adc_update_battery_voltage(void) {
	int	adcSample = adc1_get_raw(CHAN_VBAT) / 4;
	const float slope = (V2 - V1)/(ADC2 - ADC1);
	BatteryVoltage = slope*(adcSample - ADC1) + V1;
}

float adc_get_battery_voltage(void) {
	return BatteryVoltage;
}

float adc_get_battery_percentage(void) {
	float percentage = ((BatteryVoltage - battery_min) / (battery_max - battery_min)) * 100;
    if (percentage < 100)
        return percentage;
    else
        return 100.0f;
}
