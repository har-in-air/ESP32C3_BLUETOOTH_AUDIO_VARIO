#ifndef UI_H_
#define UI_H_

extern volatile bool BtnPCCPressed;
extern volatile bool BtnPCCLongPress;

void ui_indicate_uncalibrated_accel_gyro();
void ui_indicate_sleep();
void ui_indicate_fault_MS5611();
void ui_indicate_fault_MPU9250();
void ui_indicate_battery_voltage();
void ui_calibrate_accel(CALIB_PARAMS_t &calib);
void ui_calibrate_gyro(CALIB_PARAMS_t &calib);
void ui_calibrate_accel_gyro();
void ui_go_to_sleep();
void ui_btn_init();
void ui_btn_clear();

#endif
