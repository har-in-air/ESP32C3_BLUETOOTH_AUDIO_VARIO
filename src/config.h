#ifndef CONFIG_H_
#define CONFIG_H_

#if ARDUINO_USB_MODE == 0
#define pinPCCA		27	// program/configure/calibrate/audio button
#define pinAudio	14	// pwm beeper audio output
#define pinAudioEn	15	// 74HC240 output enables, active low

#define pinPwrSens	21	// detect power on/off button press
#define pinPwrCtrl	16	// power on/off

#define pinCSB		20	// CSB (ms5611)
#define pinMISO		1	// SDO ms5611 & AD0 mpu9250
#define pinNCS		3 	// NCS (mpu9250)
#define pinMOSI		4 	// SDA
#define pinSCK		5	// SCL
#define pinDRDYInt	2  	// INT
#define pinLED		0	// power-on and bluetooth active indication
#define pinGpsRx 	17
#define pinAuxTx	18
#define portAux		2

#else
#define pinPCCA		5	// program/configure/calibrate/audio button
#define pinAudio	7	// pwm beeper audio output
#define pinAudioEn	6	// 74HC240 output enables, active low

#define pinPwrSens	12	// detect power on/off button press
#define pinPwrCtrl	13	// power on/off

#define pinCSB		4	// CSB (ms5611)
#define pinMISO		9	// SDO ms5611 & AD0 mpu9250
#define pinNCS		3 	// NCS (mpu9250)
#define pinMOSI		10 	// SDA
#define pinSCK		8	// SCL
#define pinDRDYInt	2  	// INT
#define pinLED		1	// power-on and bluetooth active indication
#define pinGpsRx 	20
#define pinAuxTx	21
#define portAux	0

#endif

#define BTN_PCCA()  (digitalRead(pinPCCA) == HIGH ? 1 : 0)

#define LED_ON() 	{digitalWrite(pinLED, 0); LEDState = 1;}
#define LED_OFF() 	{digitalWrite(pinLED, 1); LEDState = 0;}

#define PWR_ON_DELAY_MS		1000UL
#define PWR_OFF_DELAY_MS	2000UL

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
#define VARIO_CROSSOVER_CPS_MAX         600

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

#define KF_ADAPT_DEFAULT	100
#define KF_ADAPT_MIN		50
#define KF_ADAPT_MAX		150

// Power-off timeout. The vario will power down
// if it does not detect climb or sink rates more than
// PWR_OFF_THRESHOLD_CPS, for the specified minutes.
#define PWR_OFF_TIMEOUT_MINUTES_DEFAULT   10
#define PWR_OFF_TIMEOUT_MINUTES_MIN       5
#define PWR_OFF_TIMEOUT_MINUTES_MAX       20

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

// #define USE_9DOF_AHRS

#define PWR_CTRL_TASK_PRIORITY	2
#define BLE_TASK_PRIORITY		3
#define WIFI_CFG_TASK_PRIORITY	3
#define VARIO_TASK_PRIORITY		(configMAX_PRIORITIES-2)

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

#define KF_ADAPT         1.0f

// Acceleration bias uncertainty is set low as the residual acceleration bias 
// (post-calibration) is expected to have low variation/drift. It is further reduced
// depending on the acceleration magnitude, as we want the acceleration bias estimate 
// to evolve ideally in a zero acceleration environment.
#define KF_ACCELBIAS_VARIANCE   	0.005f

// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE   		10.0f

// KF4 Altitude Measurement Noise Variance
#define KF_Z_MEAS_VARIANCE			200.0f

// If climbrate or sinkrate stays below this threshold for the configured
// time interval, vario goes to sleep to conserve power
#define PWR_OFF_THRESHOLD_CPS    	50

// if you find that gyro calibration fails even when you leave
// the unit undisturbed, increase this offset limit
// until you find that gyro calibration works consistently.
#define GYRO_OFFSET_LIMIT_1000DPS   200

// print debug information to the serial port for different code modules
// For revB hardware, after flashing the code and validating the calibration parameters look reasonable,
// ensure this is commented out
#define TOP_DEBUG
#if ARDUINO_USB_MODE==1
#define Serial USBSerial
#endif
#ifdef TOP_DEBUG
	#define dbg_println(x) {Serial.println x;}
	#define dbg_printf(x)  {Serial.printf x;}
	#define dbg_flush()  Serial.flush()
#else
	#define dbg_println(x)
	#define dbg_printf(x)
	#define dbg_flush()
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
// #define IMU_DEBUG
// #define BLE_DEBUG
#define ALTI_DEBUG

#endif
