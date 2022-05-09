#ifndef ADC_H_
#define ADC_H_

#define CHAN_VBAT    ADC1_CHANNEL_0 //((adc1_channel_t)0)

extern float BatteryVoltage;

void adc_init();
int adc_sample_average();
float adc_battery_voltage(int sample);
float adc_battery_voltage();

#endif
