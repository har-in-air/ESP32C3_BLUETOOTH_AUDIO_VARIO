#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "util.h"
#include "audio.h"
#include "mpu9250.h"
#include "nvd.h"


MPU9250::MPU9250() {	
	// example accel biases valid for accelerometer full scale = +/- 4G
    // actual values are computed on manual calibration and saved to flash 
    // axBias = -80
    // ayBias = -33
    // azBias = -386
    aScale_ = 1.0f/MPU9250_4G_SENSITIVITY; // accelerometer values in milli-Gs
    
    //example gyro biases for gyro full scale = 1000dps
    // actual values are calibrated each time on power up or read from flash
    //gxBias = 23;
    //gyBias = -9;
    //gzBias = 24;
    gScale_  = 1.0f/MPU9250_1000DPS_SENSITIVITY; // gyroscope values in deg/second
	}


void MPU9250::get_accel_gyro_data(float* pAccelData, float* pGyroData) {
	uint8_t buf[14];
	int16_t raw[3];
	read_bytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 14, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (float)(raw[0] - axBias_) * aScale_;
	pAccelData[1] = (float)(raw[1] - ayBias_) * aScale_;
	pAccelData[2] = (float)(raw[2] - azBias_) * aScale_;
	raw[0] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	raw[1] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
	raw[2] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);	
	pGyroData[0] = (float)(raw[0] - gxBias_) * gScale_;
	pGyroData[1] = (float)(raw[1] - gyBias_) * gScale_;
	pGyroData[2] = (float)(raw[2] - gzBias_) * gScale_;
	}

	
int MPU9250::check_id(void) {
	uint8_t whoami = read_byte(MPU9250_I2C_ADDRESS, WHO_AM_I_MPU9250);
	return (( whoami == 0x71) ? 1 : 0);
	}
	
void MPU9250::get_calib_params(CALIB_PARAMS_t &calib) {
	axBias_ = calib.axBias;
	ayBias_ = calib.ayBias;
	azBias_ = calib.azBias;
	gxBias_ = calib.gxBias;
	gyBias_ = calib.gyBias;
	gzBias_ = calib.gzBias;
#ifdef MPU9250_DEBUG
	dbg_println(("Calibration parameters from NVD"));
	dbg_printf(("Accel : axBias %d, ayBias %d, azBias %d\r\n",axBias_, ayBias_, azBias_));
	dbg_printf(("Gyro : gxBias %d, gyBias %d, gzBias %d\r\n",gxBias_, gyBias_, gzBias_));
#endif
	}


void MPU9250::sleep(void) {
	write_byte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x40);
	}

void MPU9250::config_accel_gyro(void) {
	// reset MPU9250, all registers to default settings
	write_byte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x80);
	delay(100); // Wait after reset
	// as per datasheet all registers are reset to 0 except WHOAMI and PWR_MGMT_1, 
	// so we assume reserved bits are 0
	// select best available clock source 
	write_byte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x01);
	delay(200);

	// fsync disabled, gyro bandwidth = 184Hz (with GYRO_CONFIG:fchoice_b = 00) 
	write_byte(MPU9250_I2C_ADDRESS, CONFIG, 0x01);

	// output data rate = 1000Hz/(1+1) = 500Hz
	write_byte(MPU9250_I2C_ADDRESS, SMPLRT_DIV, 0x01);

	// set gyro FS = 1000dps, fchoice_b = 00 
  // bits[4:3] = 10 
	write_byte(MPU9250_I2C_ADDRESS, GYRO_CONFIG, 0x10 );

	// Set accelerometer FS = +/-4G 
	// bits[4:3] = 01 
	write_byte(MPU9250_I2C_ADDRESS, ACCEL_CONFIG, 0x08);

	// set accelerometer BW = 184Hz
	// accel_fchoiceb = 0, a_dlpf_cfg = 1
	write_byte(MPU9250_I2C_ADDRESS, ACCEL_CONFIG2, 0x01);

	// interrupt is active high, push-pull, 50uS pulse
	write_byte(MPU9250_I2C_ADDRESS, INT_PIN_CFG, 0x10);
	// Enable data ready interrupt on INT pin
	write_byte(MPU9250_I2C_ADDRESS, INT_ENABLE, 0x01);
	delay(100);
	}


// place unit so that the sensor board accelerometer z axis points 
// vertically downwards or upwards. This is where the sensor z axis sees a static 
// acceleration of 1g / -1g. In this orientation the ax and ay values are 
// the offsets for a 0g environment. 
// Repeat this calibration a few times with the debug serial monitor to check the 
// consistency of the calibration offsets. The board MUST be in a 1g static acceleration 
// environment for this calibration, i.e. at rest, no vibrations etc.

#define ACCEL_NUM_AVG_SAMPLES	100

void MPU9250::calibrate_accel(CALIB_PARAMS_t &calib){
	uint8_t buf[6];
	int16_t x,y,z;
	int32_t axAccum, ayAccum, azAccum;
	axAccum = ayAccum = azAccum = 0;
	for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
		read_bytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 6, buf);
		x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
		y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
		z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
		axAccum += (int32_t) x;
		ayAccum += (int32_t) y;
		azAccum += (int32_t) z;
		delay(2);
		}
	x = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
	y = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
	z = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);
#ifdef MPU9250_DEBUG
	dbg_printf(("ax = %d  ay = %d  az = %d\r\n", x, y, z));
#endif

	calib.axBias = axBias_ = x;
	calib.ayBias = ayBias_ = y;
	calib.azBias = azBias_ = z > 0 ? z - (int16_t)(1000.0f*MPU9250_4G_SENSITIVITY) : z + (int16_t)(1000.0f*MPU9250_4G_SENSITIVITY);

#ifdef MPU9250_DEBUG
  dbg_printf(("axBias = %d\r\n", (int)axBias_));
  dbg_printf(("ayBias = %d\r\n", (int)ayBias_));
  dbg_printf(("azBias = %d\r\n", (int)azBias_));
#endif
	}

#define GYRO_NUM_CALIB_SAMPLES			50
	
int MPU9250::calibrate_gyro(CALIB_PARAMS_t &calib){
	uint8_t buf[6];
	int16_t gx,gy,gz;
	int32_t gxAccum, gyAccum, gzAccum;
	int foundBadData;
	int numTries = 1;
	do {
		delay(500);
		foundBadData = 0;
		gxAccum = gyAccum = gzAccum = 0;
		for (int inx = 0; inx < GYRO_NUM_CALIB_SAMPLES; inx++){
			read_bytes(MPU9250_I2C_ADDRESS, GYRO_XOUT_H, 6, buf);
			gx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
			gy = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
			gz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
			
			// if a larger than expected gyro bias is measured, 
			// assume the unit was disturbed and try again after a short delay, upto 10 times
			// Note : if gyro calibration fails even when the unit is not disturbed, increase
			// GYRO_OFFSET_LIMIT_1000DPS
			if ((ABS(gx) > GYRO_OFFSET_LIMIT_1000DPS) || 
			    (ABS(gy) > GYRO_OFFSET_LIMIT_1000DPS) || 
				(ABS(gz) > GYRO_OFFSET_LIMIT_1000DPS)) {
				foundBadData = 1;
				// generate a low tone pulse each time calibration fails. If you hear this even when the unit is left undisturbed for calibration,
				// the MPU9250 gyro has a high bias on one or more axes, you will need to increase the configuration parameter gyroOffsetLimit1000DPS. 
				audio_generate_tone(200, 300); 
				break;
				}  
			gxAccum  += (int32_t) gx;
			gyAccum  += (int32_t) gy;
			gzAccum  += (int32_t) gz;
			delay(2);
			}
		} while (foundBadData && (++numTries < 10));

	// update gyro biases only if calibration succeeded, else use the last saved values from flash memory. Valid scenario for
	// gyro calibration failing is when you turn on the unit while flying. So not a big deal.
    if (!foundBadData) {		
		  calib.gxBias = gxBias_ =  (int16_t)( gxAccum / GYRO_NUM_CALIB_SAMPLES);
		  calib.gyBias = gyBias_ =  (int16_t)( gyAccum / GYRO_NUM_CALIB_SAMPLES);
		  calib.gzBias = gzBias_ =  (int16_t)( gzAccum / GYRO_NUM_CALIB_SAMPLES);		
		  }
#ifdef MPU9250_DEBUG
	dbg_printf(("Num Tries = %d\r\n",numTries));
	dbg_printf(("gxBias = %d\r\n",gxBias_));
	dbg_printf(("gyBias = %d\r\n",gyBias_));
	dbg_printf(("gzBias = %d\r\n",gzBias_));
#endif
	return (foundBadData ? 0 : 1);
	}


void MPU9250::write_byte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t d) {
	Wire.beginTransmission(deviceAddress);  
	Wire.write(registerAddress);
	Wire.write(d);           
	Wire.endTransmission();     
	}

uint8_t MPU9250::read_byte(uint8_t deviceAddress, uint8_t registerAddress){
	uint8_t d; 
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.endTransmission(false); // restart
	Wire.requestFrom(deviceAddress, (uint8_t) 1);
	d = Wire.read();
	return d;
	}

int MPU9250::read_bytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t * dest) {
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.endTransmission(false); // restart
	int cnt = 0;
	Wire.requestFrom(deviceAddress, count);
	while (Wire.available())  {
		dest[cnt++] = Wire.read();
		}
	return cnt;
	}
