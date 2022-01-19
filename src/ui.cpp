#include <Arduino.h>
#include <Ticker.h>
#include "config.h"
#include "audio.h"
#include "adc.h"
#include "nvd.h"
#include "ui.h"
#include "mpu9250.h"

Ticker     Tickr;

extern MPU9250 Mpu9250;

volatile uint32_t BtnPCCState;
volatile bool BtnPCCPressed = false;
volatile bool BtnPCCLongPress = false;
	
void IRAM_ATTR btn_debounce() {
	BtnPCCState = ((BtnPCCState<<1) | ((uint32_t)BTN_PCC()) );
	if ((BtnPCCState | 0xFFFFFFF0) == 0xFFFFFFF8) {
		BtnPCCPressed = true;
		}    
	if (BtnPCCState == 0) {
		BtnPCCLongPress = true;
		}
	}

	
void ui_btn_init() {
 	Tickr.attach_ms(25, btn_debounce);
	ui_btn_clear();
	}
		

void ui_btn_clear() {
	BtnPCCPressed  = false;
	BtnPCCLongPress = false;
	}

	
// !! Accelerometer calibration is REQUIRED for normal vario operation. !!
// If flash was completely erased, or Mpu9250 calibration data in flash was never initialized, or 
// imu calibration data is corrupt, the accel and gyro biases are set to 0. Uncalibrated 
// state is indicated with a continuous sequence of alternating high and low beeps for 5 seconds.
void ui_indicate_uncalibrated_accel_gyro() {
	for (int cnt = 0; cnt < 5; cnt++) {
		audio_generate_tone(UNCALIBRATED_TONE_HZ, 500); 
		audio_generate_tone(UNCALIBRATED_TONE_HZ/2, 500);
		}
	}

	
// "no-activity" timeout sleep is indicated with a series of descending
// tones. If you do hear this, switch off the vario as there is still
// residual current draw from the circuit components in sleep mode
void ui_indicate_sleep() {
	audio_generate_tone(2000,1000); 
	audio_generate_tone(1000,1000);
	audio_generate_tone(500, 1000);
	audio_generate_tone(250, 1000);
	}

// problem with MS5611 calibration CRC, assume communication error or bad device. 
void ui_indicate_fault_MS5611() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio_generate_tone(MS5611_ERROR_TONE_HZ, 1000); 
		delay(100);
		}
	}

// problem reading MPU9250 ID, assume communication error or bad device. 
void ui_indicate_fault_MPU9250() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio_generate_tone(MPU9250_ERROR_TONE_HZ, 1000); 
		delay(100);
		}
	}

void ui_indicate_battery_voltage() {
	int numBeeps;
	int adcVal = adc_sample_average();
	float fbv = adc_battery_voltage(adcVal);
	dbg_printf(("\r\nBattery voltage = %.2fV\r\n", fbv ));

	if (fbv >= 4.0f) numBeeps = 5;
	else
	if (fbv >= 3.9f) numBeeps = 4;
	else
	if (fbv >= 3.7f) numBeeps = 3;
	else
	if (fbv >= 3.6f) numBeeps = 2;
	else numBeeps = 1;
	while (numBeeps--) {
		audio_generate_tone(BATTERY_TONE_HZ, 300);
		delay(300);
		}
	}
   

void ui_calibrate_accel(CALIB_PARAMS_t &calib) {    
    // acknowledge calibration button press with long tone
    audio_generate_tone(CALIBRATING_TONE_HZ, 3000);
    dbg_println(("-- Accelerometer calibration --"));
    dbg_println(("Place vario on a level surface with accelerometer z axis vertical and leave it undisturbed"));
    dbg_println(("You have 10 seconds, counted down with rapid beeps from 50 to 0"));
    for (int inx = 0; inx < 50; inx++) {
		delay(200); 
		dbg_println((50-inx));
		audio_generate_tone(CALIBRATING_TONE_HZ, 50);
		}
    dbg_println(("\r\nCalibrating accelerometer"));
    Mpu9250.calibrate_accel(calib);
    dbg_println(("Accelerometer calibration done"));
    nvd_calib_store(calib);
    }


void ui_calibrate_gyro(CALIB_PARAMS_t &calib) {    
	dbg_println(("\r\nCalibrating gyro"));
	// normal power-on operation flow, always attempt to calibrate gyro. If calibration isn't possible because 
	// the unit is continuously disturbed (e.g. you turned on the unit while already flying), indicate this and
	// use the last saved gyro biases. Otherwise, save the new gyro biases to flash memory
	if (Mpu9250.calibrate_gyro(calib)) {
		dbg_println(("Gyro calibration OK"));
		audio_generate_tone(CALIBRATING_TONE_HZ, 1000);
		nvd_calib_store(calib);
		}
	else { 
		dbg_println(("Gyro calibration failed"));
		audio_generate_tone(CALIBRATING_TONE_HZ, 1000);
		delay(500);
		audio_generate_tone(CALIBRATING_TONE_HZ/2, 1000);
		}
	}


// Vario will attempt to calibrate gyro each time on power up. If the vario is disturbed, it will
// use the last saved gyro calibration values.
// The software delays a few seconds so that the unit can be left undisturbed for gyro calibration.
// This delay is indicated with a series of 10 short beeps. While it is beeping, if you press the
// PCC button, the unit will calibrate the accelerometer first and then the gyro.
// As soon as you hear the long confirmation tone, release the button and
// put the unit in accelerometer calibration position resting undisturbed on a horizontal surface 
// with the accelerometer +z axis pointing vertically downwards. You will have some time 
// to do this, indicated by a series of beeps. After calibration, the unit will generate another 
// tone, save the calibration parameters to flash, and continue with normal vario operation
void ui_calibrate_accel_gyro() {  
	boolean bCalibrateAccel = false;
    // load the accel & gyro calibration parameters from the non-volatile data structure
  	if ((Calib.axBias == 0) && (Calib.ayBias == 0) && (Calib.azBias == 0)) {
    	dbg_println(("! Uncalibrated accelerometer !"));
    	ui_indicate_uncalibrated_accel_gyro(); 
		bCalibrateAccel = true;    
    	}
	if (bCalibrateAccel == true) {  
    	dbg_println(("Starting accelerometer calibration"));
		ui_calibrate_accel(Calib);
		bCalibrateAccel = false;
		}	
	dbg_println(("Counting down to gyro calibration"));
	dbg_println(("Press the PCC button to enforce accelerometer calibration first"));
	for (int inx = 0; inx < 10; inx++) {
		delay(500); 
		dbg_println((10-inx));
		audio_generate_tone(CALIBRATING_TONE_HZ, 50); 
		if (digitalRead(pinPCC) == 0) {
			bCalibrateAccel = true;
			dbg_println(("PCC button pressed"));
			break;
			}
		}
	if (bCalibrateAccel == true) {  
		ui_calibrate_accel(Calib);
		}
	ui_calibrate_gyro(Calib);
	}
	

// residual current draw in sleep mode is the sum of ESP8266 deep sleep mode current,
// MPU9250 sleep mode current, MS5611 standby current, quiescent current of voltage
// regulators, and miscellaneous current through resistive paths e.g. the
// ADC voltage divider.
void ui_go_to_sleep() {
	audio_set_frequency(0); // switch off pwm audio 
	Mpu9250.sleep(); // put MPU9250 in sleep mode
	//adc_power_off();
	esp_deep_sleep_start();
	}

	
