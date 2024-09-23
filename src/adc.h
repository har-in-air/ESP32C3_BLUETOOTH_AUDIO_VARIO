#ifndef ADC_H_
#define ADC_H_

void adc_init(void);
void adc_update_battery_voltage(void);
float adc_get_battery_voltage(void);

#endif
