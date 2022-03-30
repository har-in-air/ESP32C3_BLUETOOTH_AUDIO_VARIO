#ifndef UI_H_
#define UI_H_

#include "config.h"

extern volatile bool BtnPCCAPressed;
extern volatile bool BtnPCCALongPress;

void ui_indicate_uncalibrated_imu();
void ui_indicate_power_off();
void ui_indicate_fault_MS5611();
void ui_indicate_fault_MPU9250();
void ui_indicate_battery_voltage();
void ui_calibrate_accel(CALIB_PARAMS_t &calib);
void ui_calibrate_gyro(CALIB_PARAMS_t &calib);
#ifdef USE_9DOF_AHRS
void ui_calibrate_accel_gyro_mag();
void ui_calibrate_mag(CALIB_PARAMS_t &calib);
#else 
void ui_calibrate_accel_gyro();
#endif

void ui_btn_init();
void ui_btn_clear();

#endif
