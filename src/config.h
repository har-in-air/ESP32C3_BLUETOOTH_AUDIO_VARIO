#ifndef CONFIG_H_
#define CONFIG_H_

#define pinPCCA		9	// program/configure/calibrate/audio button
#define pinAudio	4	// pwm beeper audio output
#define pinAudioEn	3	// 74HC240 output enables, active low

#define pinPwrSens	1	// detect power on/off button press
#define pinPwrCtrl	2	// power on/off

#define pinCSB		5	// CSB (ms5611)
#define pinMISO		7	// SDO ms5611 & AD0 mpu9250
#define pinNCS		10 	// NCS (mpu9250)
#define pinMOSI		18 	// SDA
#define pinSCK		19	// SCL
#define pinDRDYInt	6  	// INT
#define pinLED		8	// power-on and bluetooth active indication

#define BTN_PCCA()  (digitalRead(pinPCCA) == HIGH ? 1 : 0)

#define LED_ON() 	{digitalWrite(pinLED, 0); LEDState = 1;}
#define LED_OFF() 	{digitalWrite(pinLED, 1); LEDState = 0;}

#define PWR_ON_DELAY_MS		1000
#define PWR_OFF_DELAY_MS	2000

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	-20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-250
#define VARIO_SINK_THRESHOLD_CPS_MIN    	-400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	-100

// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         800

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

// adaptive uncertainty injection
#define KF_ADAPT_DEFAULT     100
#define KF_ADAPT_MIN         50
#define KF_ADAPT_MAX         150

// Power-off timeout. The vario will power down
// if it does not detect climb or sink rates more than
// PWR_OFF_THRESHOLD_CPS, for the specified minutes.
#define PWR_OFF_TIMEOUT_MINUTES_DEFAULT   5
#define PWR_OFF_TIMEOUT_MINUTES_MIN       5
#define PWR_OFF_TIMEOUT_MINUTES_MAX       15

// audio feedback tones
#define BATTERY_TONE_HZ			400
#define CALIBRATING_TONE_HZ		800
#define UNCALIBRATED_TONE_HZ	2000
#define MPU9250_ERROR_TONE_HZ	200 
#define MS5611_ERROR_TONE_HZ	2500

// BLE LK8EX1 transmission is enabled as default
#define BLE_DEFAULT  true

////////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

//#define USE_9DOF_AHRS

#define PWR_CTRL_TASK_PRIORITY	1
#define BLE_TASK_PRIORITY		2
#define WIFI_CFG_TASK_PRIORITY	2
#define VARIO_TASK_PRIORITY		(configMAX_PRIORITIES-1)

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// Acceleration bias uncertainty is set low as the residual acceleration bias 
// (post-calibration) is expected to have low variation/drift. It is further reduced
// depending on the acceleration magnitude, as we want the
// acceleration bias estimate to evolve only in close to zero 
// acceleration environment.
#define KF_ACCELBIAS_VARIANCE   	0.005f

// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE   		2.0f

// KF4 Altitude Measurement Noise Variance
#define KF_Z_MEAS_VARIANCE			200.0f

// injects additional uncertainty depending on magnitude of acceleration
// helps respond quickly to large accelerations while heavily filtering
// in low or no acceleration situations.  Range : 0.0 (no adaptation)
// to 1.0 (max adaptive factor)
#define KF_ADAPTIVE_ACCEL_FACTOR	1.0f

// If climbrate or sinkrate stays below this threshold for the configured
// time interval, vario goes to sleep to conserve power
#define PWR_OFF_THRESHOLD_CPS    	50

// if you find that gyro calibration fails even when you leave
// the unit undisturbed, increase this offset limit
// until you find that gyro calibration works consistently.
#define GYRO_OFFSET_LIMIT_1000DPS   50

// print debug information to the serial port for different code modules

#define TOP_DEBUG
#ifdef TOP_DEBUG
	#define dbg_println(x) {Serial.println x;}
	#define dbg_printf(x)  {Serial.printf x;}
#else
	#define dbg_println(x)
	#define dbg_printf(x)
#endif
// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define KF_DEBUG
#define VARIO_DEBUG
#define NVD_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
//#define IMU_DEBUG
//#define PERF_DEBUG
//#define BLE_DEBUG

#endif
