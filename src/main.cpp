#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "util.h"
#include "imu.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "kalmanfilter4.h"
#include "audio.h"
#include "vaudio.h"
#include "adc.h"
#include "nvd.h"
#include "ringbuf.h"
#include "wifi_cfg.h"
#include "ui.h"
#include "ble_uart.h"

const char* FwRevision = "0.90";

MPU9250    Mpu9250;
MS5611     Ms5611;

int      	AppMode;
boolean 	bWebConfigure = false;

float AccelmG[3];  // milli-Gs
float GyroDps[3];  // degrees/second
float KfAltitudeCm = 0.0f; // kalman filtered altitude in cm
float KfClimbrateCps = 0.0f;  // kalman filtered climb/sink rate in cm/s

uint32_t TimePreviousUs; // time markers
uint32_t TimeNowUs;
float 	 ImuTimeDeltaUSecs; // time between imu samples, in microseconds
float 	 KfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

int SleepCounter;
int BaroCounter;
int SleepTimeoutSecs;

#if (CFG_LANTERN == true)
int LEDPwmLkp[4] = {LANTERN_DIM, LANTERN_LOW, LANTERN_MID, LANTERN_HI};
int LanternState;
#endif

volatile int DrdyCounter = 0;
volatile boolean DrdyFlag = false;

static void IRAM_ATTR drdy_interrupt_handler();
static void time_init();

// pinPCC (GPIO9) has an external 10K pullup resistor to VCC
// pressing the button  will ground the pin.
// This button has three different functions : program, configure, and calibrate (PCC)
// 1. (Program)
//    Power on the unit with PCC button pressed. Or with power on, keep 
//    PCC pressed and momentarily press the reset button.
//    This will put the ESP32-C3 into programming mode, and you can flash 
//    the application code from the Arduino IDE.
// 2. (WiFi Configuration)
//    After normal power on, immediately press PCC and keep it pressed. 
//    Wait until you hear a low tone, then release. The unit will now be in WiFi configuration
//    configuration mode. 
// 3. (Calibrate)
//    After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the PGCC button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer re-calibration is required if the acceleration calibration values in 
//    flash were never written, or if the entire flash has been erased.


// handles data ready interrupt from MPU9250 (every 2ms)
static void IRAM_ATTR drdy_interrupt_handler() {
	DrdyFlag = true;
	DrdyCounter++;
	}	


// setup time markers for Mpu9250, Ms5611 and kalman filter
static void time_init() {
	TimeNowUs = TimePreviousUs = micros();
	}


inline void time_update(){
	TimeNowUs = micros();
	ImuTimeDeltaUSecs = TimeNowUs > TimePreviousUs ? (float)(TimeNowUs - TimePreviousUs) : 2000.0f; // if rollover use expected time difference
	TimePreviousUs = TimeNowUs;
	}


void setup_vario() {
	dbg_println(("Vario mode"));
 	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(400000); // set i2c clock frequency to 400kHz, AFTER Wire.begin()
	delay(100);
	dbg_println(("\r\nChecking communication with MS5611"));
	if (!Ms5611.read_prom()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		Serial.flush();
		ui_indicate_fault_MS5611(); 
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MS5611 OK"));
  
	dbg_println(("\r\nChecking communication with MPU9250"));
	if (!Mpu9250.check_id()) {
		dbg_println(("Error reading Mpu9250 WHO_AM_I register"));
		Serial.flush();
		ui_indicate_fault_MPU9250();
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MPU9250 OK"));
    
	DrdyCounter = 0;
	DrdyFlag = false;
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt (GPIO10) which has an external 10K pull-down resistor
	pinMode(pinDRDYInt, INPUT); 
	attachInterrupt(pinDRDYInt, drdy_interrupt_handler, RISING);

	// configure MPU9250 to start generating gyro and accel data  
	Mpu9250.config_accel_gyro();

	// calibrate gyro (and accel if required)
	ui_calibrate_accel_gyro();
	delay(50);  
	  
	dbg_println(("\r\nMS5611 config"));
	Ms5611.reset();
	Ms5611.get_calib_coefficients(); // load MS5611 factory programmed calibration data
	Ms5611.averaged_sample(4); // get an estimate of starting altitude
	Ms5611.init_sample_state_machine(); // start the pressure & temperature sampling cycle

	dbg_println(("\r\nKalmanFilter config"));
	// initialize kalman filter with Ms5611 estimated altitude, estimated initial climbrate = 0.0
	kalmanFilter4_configure((float)Config.kf.zMeasVariance, 1000.0f*(float)Config.kf.accelVariance, true, Ms5611.altitudeCmAvg, 0.0f, 0.0f);

	vaudio_config();  
	time_init();
	KfTimeDeltaUSecs = 0.0f;
	BaroCounter = 0;
	SleepTimeoutSecs = 0;
	ringbuf_init(); 
	SleepCounter = 0;
	if (Config.misc.bleEnable){
		ble_uart_init();
		dbg_println(("\r\nStarting Vario with Bluetooth LE LK8EX1 messages @ 10Hz\r\n"));
		}
	else {
		dbg_println(("\r\nStarting Vario with Bluetooth LE disabled\r\n"));  
		}
	}

#if (CFG_LANTERN == true)   
void setup_lantern() {
	ledcSetup(LED_CHANNEL, LED_FREQUENCY, LED_RESOLUTION);
	ledcAttachPin(pinLED, LED_CHANNEL);	
    dbg_println(("Lantern mode"));
    LanternState = 0;
    ledcWrite(LED_CHANNEL, LEDPwmLkp[LanternState]);
    }
#endif



void setup() {
	pinMode(pinPCC, INPUT); //  Program/Configure/Calibrate Button
	wifi_off(); // turn off radio to save power

#ifdef TOP_DEBUG    
	Serial.begin(115200);
#endif
  
	dbg_printf(("\r\n\r\nESP32-C3 BLUETOOTH VARIO compiled on %s at %s\r\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\r\n", FwRevision));
	dbg_println(("\r\nLoad non-volatile configuration and calibration data from flash"));  
	nvd_config_load(Config);
	nvd_calib_load(Calib);

	if (!LittleFS.begin()){
		dbg_println(("Error mounting LittleFS, restarting..."));
		delay(1000);
		ESP.restart();
		}   

	bWebConfigure = false;
	dbg_println(("To start web configuration mode, press and hold the PCC button"));
	dbg_println(("until you hear a low-frequency tone. Then release the button"));
	for (int cnt = 0; cnt < 8; cnt++) {
		dbg_println((8-cnt));
		delay(500);
		if (digitalRead(pinPCC) == 0) {
			bWebConfigure = true;
			break;
			}
		}
	if (bWebConfigure == true) {
		dbg_println(("Web configuration mode"));
		// 3 second long tone with low frequency to indicate unit is now in web server configuration mode.
		// After you are done with web configuration, switch off the vario as the wifi radio
		// consumes a lot of power.
		audio_generate_tone(200, 3000);
		wificfg_ap_server_init(); 
		}
  	else {
		dbg_println(("Vario mode"));
    	ui_indicate_battery_voltage();
    	switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			setup_vario();
			break;

#if (CFG_LANTERN == true)
			case APP_MODE_LANTERN :
			setup_lantern();
			break;
#endif            
			}
		}
	ui_btn_init();	
	}


void vario_loop() {
	if (DrdyFlag == true) {
		// MPU9250 500Hz ODR => 2mS sample interval
		DrdyFlag = false;
		time_update();
		#ifdef CCT_DEBUG    
		uint32_t marker = micros(); // set marker for estimating the time taken to read and process the data (needs to be < 2mS !!)
		#endif    
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		Mpu9250.get_accel_gyro_data(AccelmG, GyroDps); 

		// We arbitrarily decide that the CJMCU-117 board silkscreen Y points "forward" or "north"  (the side with the HM-11), 
		// silkscreen X points "right" or "east", and silkscreen Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in our AHRS algorithm implementation.
		// The required mapping from sensor samples to NED frame for our specific board orientation is : 
		// gxned = gx, gyned = gy, gzned = -gz (clockwise rotations about the axis must result in +ve readings on the axis)
		// axned = ay, ayned = ax, azned = az (when the axis points down, axis reading must be +ve)
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float accelMagnitudeSquared = AccelmG[0]*AccelmG[0] + AccelmG[1]*AccelmG[1] + AccelmG[2]*AccelmG[2];
		int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
        float dtIMU = ImuTimeDeltaUSecs/1000000.0f;
        float gxned = DEG_TO_RAD*GyroDps[0];
        float gyned = DEG_TO_RAD*GyroDps[1];
        float gzned = -DEG_TO_RAD*GyroDps[2];
        float axned = AccelmG[1];
        float ayned = AccelmG[0];
        float azned = AccelmG[2];
		imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gxned, gyned, gzned, axned, ayned, azned);
		float gCompensatedAccel = imu_gravity_compensated_accel(axned, ayned, azned, Q0, Q1, Q2, Q3);
		ringbuf_add_sample(gCompensatedAccel);  
		BaroCounter++;
		KfTimeDeltaUSecs += ImuTimeDeltaUSecs;
		int32_t climbrate;
		if (BaroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			BaroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = Ms5611.sample_state_machine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_average_newest_samples(10); 
				float dtKF = KfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4_predict(dtKF);
				kalmanFilter4_update(Ms5611.altitudeCm, zAccelAverage, (float*)&KfAltitudeCm, (float*)&KfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				KfTimeDeltaUSecs = 0.0f;
				climbrate = F_TO_I(KfClimbrateCps);
				vaudio_tick_handler(climbrate); // audio feedback handler
				if (ABS(climbrate) > SLEEP_THRESHOLD_CPS) { 
					// reset sleep timeout watchdog if there is significant vertical motion
					SleepTimeoutSecs = 0;
					}
				else
				if (SleepTimeoutSecs >= (Config.misc.sleepTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, put MPU9250 and ESP32-C3 to sleep to minimize current draw"));
					Serial.flush();
					ui_indicate_sleep(); 
					ui_go_to_sleep();
					}   
				}
			}
			
	#ifdef CCT_DEBUG      
		uint32_t elapsedUs =  micros() - marker; // calculate time  taken to read and process the data, must be less than 2mS
	#endif
		if (DrdyCounter >= 50) {
			DrdyCounter = 0; // 0.1 second elapsed
			if (Config.misc.bleEnable) {
				int adcVal = analogRead(A0);
				float batVoltage = adc_battery_voltage(adcVal);
				float faltM = KfAltitudeCm/100.0f;
				int altM =  F_TO_I(faltM);
				ble_uart_transmit_LK8EX1(altM, climbrate, batVoltage);				
				}
			SleepCounter++;
			if (SleepCounter >= 10) {
				SleepCounter = 0;
				SleepTimeoutSecs++;
				#ifdef IMU_DEBUG
				float yaw, pitch, roll;
				imu_quaternion_to_yaw_pitch_roll(Q0,Q1,Q2,Q3, &yaw, &pitch, &roll);
				// Pitch is positive for clockwise rotation about the NED frame +Y axis
				// Roll is positive for clockwise rotation about the NED frame +X axis
				// Yaw is positive for clockwise rotation about the NED frame +Z axis
				// Magnetometer isn't used, so yaw is initialized to 0 for the "forward" direction of the case on power up.
				dbg_printf(("\r\nY = %d P = %d R = %d\r\n", (int)yaw, (int)pitch, (int)roll));
				dbg_printf(("ba = %d ka = %d kv = %d\r\n",(int)Ms5611.altitudeCm, (int)KfAltitudeCm, (int)KfClimbrateCps));
				#endif     
				#ifdef CCT_DEBUG      
                // The raw IMU data rate is 500Hz, i.e. 2000uS between Data Ready Interrupts
                // We need to read the MPU9250 data, MS5611 data and finish all computations
                // and actions well within this interval.
                // last checked, < 620 uS @ 80MHz clock
				dbg_printf(("Elapsed %dus\r\n", (int)elapsedUs)); 
				#endif
				}
			}
		}	
#if (CFG_LANTERN == true)
	if (BtnPCCLongPress == true) {
		AppMode = APP_MODE_LANTERN;
		setup_lantern();
		delay(500);
		ui_btn_clear();    
		}  
#endif        
	}


#if (CFG_LANTERN == true)
void lantern_loop() {
	int count;
	if (BtnPCCPressed) {
		ui_btn_clear();
		LanternState++;
		if (LanternState > 4) {
			LanternState = 0;
			}
		if (LanternState < 4) {
		    ledcWrite(LED_CHANNEL, LEDPwmLkp[LanternState]);
			}
		}
	if (LanternState == 4) {
		// flash S.O.S. pattern
		count = 3;
		while (count--) {
			if (BtnPCCPressed) return;
		    ledcWrite(LED_CHANNEL, LANTERN_HI);
			delay(50);
		    ledcWrite(LED_CHANNEL, 0);
			delay(400);
			}
		count = 12;
		while (count--) {
			if (BtnPCCPressed) return;
			delay(50);        
			}
		count = 3;
		while (count--) {
			if (BtnPCCPressed) return;
		    ledcWrite(LED_CHANNEL, LANTERN_HI);
			delay(1000);
		    ledcWrite(LED_CHANNEL, 0);
			delay(400);
			}
		count = 12;
		while (count--) {
			if (BtnPCCPressed) return;
			delay(50);        
			}
		count = 3;
		while (count--) {
			if (BtnPCCPressed) return;
		    ledcWrite(LED_CHANNEL, LANTERN_HI);
			delay(50);
		    ledcWrite(LED_CHANNEL, 0);
			delay(400);
			}
		count = 80;
		while (count--) {
			if (BtnPCCPressed) return;
			delay(50);
			}    
		}
	}
#endif


void loop(){
	if (bWebConfigure == true) {
		// nothing to do here, async web server runs in its own thread
		}
	else { 
		switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			vario_loop();
			break;

#if (CFG_LANTERN == true)
			case APP_MODE_LANTERN :
			lantern_loop();
			break;      
#endif            
			}
		} 
	}
