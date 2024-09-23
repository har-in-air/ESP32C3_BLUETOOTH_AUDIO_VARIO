#ifndef ADC_H_
#define ADC_H_

extern void adc_init(void);
extern void adc_update_battery_voltage(void);
extern float adc_get_battery_voltage(void);
extern float adc_get_battery_percentage(void);

#endif
