#include <Arduino.h>
#include "config.h"
#include "spi.h"
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
    aScale = 1.0f/MPU9250_4G_SENSITIVITY; // accelerometer values in milli-Gs
    
    //example gyro biases for gyro full scale = 1000dps
    // actual values are calibrated each time on power up or read from flash
    //gxBias = 23;
    //gyBias = -9;
    //gzBias = 24;
    gScale  = 1.0f/MPU9250_1000DPS_SENSITIVITY; // gyroscope values in deg/second
	}


void MPU9250::get_accel_gyro_data(float* pAccelData, float* pGyroData) {
	uint8_t buf[14];
	int16_t x,y,z;
	spi_read_buffer(spiImu, R_ACCEL_XOUT_H  | SPI_READ, 14, buf);
	x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (float)(x - axBias) * aScale;
	pAccelData[1] = (float)(y - ayBias) * aScale;
	pAccelData[2] = (float)(z - azBias) * aScale;
	x = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	y = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
	z = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);	
	pGyroData[0] = (float)(x - gxBias) * gScale;
	pGyroData[1] = (float)(y - gyBias) * gScale;
	pGyroData[2] = (float)(z - gzBias) * gScale;
	}


void MPU9250::get_accel_gyro_mag_data(float* pAccelData, float* pGyroData, float* pMagData) {
	uint8_t buf[20];
	int16_t x,y,z;
	spi_read_buffer(spiImu, R_ACCEL_XOUT_H  | SPI_READ, 20, buf);
	x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (float)(x - axBias) * aScale;
	pAccelData[1] = (float)(y - ayBias) * aScale;
	pAccelData[2] = (float)(z - azBias) * aScale;
	
	x = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	y = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
	z = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);	
	pGyroData[0] = (float)(x - gxBias) * gScale;
	pGyroData[1] = (float)(y - gyBias) * gScale;
	pGyroData[2] = (float)(z - gzBias) * gScale;

	// mag data is little endian	
	x = (int16_t)(((uint16_t)buf[15] << 8) | (uint16_t)buf[14]);
	y = (int16_t)(((uint16_t)buf[17] << 8) | (uint16_t)buf[16]);
	z = (int16_t)(((uint16_t)buf[19] << 8) | (uint16_t)buf[18]);	
	int xi = ((int)x * (asaX+128))/256;
	int yi = ((int)y * (asaY+128))/256;
	int zi = ((int)z * (asaZ+128))/256;

	pMagData[0] = (float)(xi - mxBias) * mxScale;
	pMagData[1] = (float)(yi - myBias) * myScale;
	pMagData[2] = (float)(zi - mzBias) * mzScale;	
	}


int MPU9250::check_id(void) {
	uint8_t id = spi_read_register(spiImu, R_WHO_AM_I  | SPI_READ);
	dbg_printf(("MPU9250 ID = %02X, expected 0x71\n", id));
	return (id == 0x71 ? 1 : 0);
	}
	

void MPU9250::get_calib_params(CALIB_PARAMS_t &calib) {
	axBias = calib.axBias;
	ayBias = calib.ayBias;
	azBias = calib.azBias;
	gxBias = calib.gxBias;
	gyBias = calib.gyBias;
	gzBias = calib.gzBias;
	mxBias = calib.mxBias;
	myBias = calib.myBias;
	mzBias = calib.mzBias;
	mxScale = calib.mxScale;
	myScale = calib.myScale;
	mzScale = calib.mzScale;

#ifdef MPU9250_DEBUG
	dbg_println(("Calibration parameters from NVD"));
	dbg_printf(("axBias %d, ayBias %d, azBias %d\n",axBias, ayBias, azBias));
	dbg_printf(("gxBias %d, gyBias %d, gzBias %d\n",gxBias, gyBias, gzBias));
	dbg_printf(("mxBias %d, myBias %d, mzBias %d\n",mxBias, myBias, mzBias));
	dbg_printf(("mxScale %f, myScale %f, mzScale %f\n",mxScale, myScale, mzScale));
#endif
	}


void MPU9250::sleep(void) {
	spi_write_register(spiImu, R_PWR_MGMT_1, 0x40);
	}


void MPU9250::config_accel_gyro(void) {
	// reset MPU9250, all registers to default settings
	spi_write_register(spiImu, R_PWR_MGMT_1, PWR_RESET);	
	delay(100); // Wait after reset
	// as per datasheet all registers are reset to 0 except WHOAMI and PWR_MGMT_1, 
	// so we assume reserved bits are 0
	// select best available clock source 
	spi_write_register(spiImu, R_PWR_MGMT_1, CLOCK_SEL_PLL);	
	delay(200);

	// fsync disabled, gyro bandwidth = 184Hz (with GYRO_CONFIG:fchoice_b = 00) 
	spi_write_register(spiImu, R_CONFIG, 0x01);	

	// output data rate = 1000Hz/(1+1) = 500Hz
	spi_write_register(spiImu, R_SMPLRT_DIV, 0x01);	

	// set gyro FS = 1000dps, fchoice_b = 00 
    // bits[4:3] = 10 
	spi_write_register(spiImu, R_GYRO_CONFIG, 0x10);	

	// Set accelerometer FS = +/-4G 
	// bits[4:3] = 01 
	spi_write_register(spiImu, R_ACCEL_CONFIG, 0x08);	

	// set accelerometer BW = 184Hz
	// accel_fchoiceb = 0, a_dlpf_cfg = 1
	spi_write_register(spiImu, R_ACCEL_CONFIG2, 0x01);	

	// interrupt is active high, push-pull, 50uS pulse
	spi_write_register(spiImu, R_INT_PIN_CFG, 0x10);	

	// Enable data ready interrupt on INT pin
	spi_write_register(spiImu, R_INT_ENABLE, 0x01);	
	delay(100);
	}


void MPU9250::config_accel_gyro_mag(void) {
	uint8_t buffer[3];
	// reset MPU9250, all registers to default settings
	spi_write_register(spiImu, R_PWR_MGMT_1, PWR_RESET);	
	delay(100); // Wait after reset
	// select gyro as clock source
	spi_write_register(spiImu, R_PWR_MGMT_1, CLOCK_SEL_PLL);	
	// enable i2c master mode
	spi_write_register(spiImu, R_USER_CTRL, I2C_MST_EN);	
	// set i2c bus clock to 400kHz
	spi_write_register(spiImu, R_I2C_MST_CTRL, I2C_MST_CLK);	

	read_AK8963_registers(AK8963_WHO_AM_I, 1, buffer);
	dbg_printf(("AK8963 id = %02X, expected 0x48", buffer[0]));
	// set AK8963 to power down
	spi_write_register(spiImu, R_I2C_MST_CTRL, I2C_MST_CLK);	
	// power down AK8963
	write_AK8963_register(AK8963_CNTL1,AK8963_PWR_DOWN);
	// reset mpu9250
	spi_write_register(spiImu, R_PWR_MGMT_1, PWR_RESET);	
    ets_delay_us(1000);
	// reset AK8963
	write_AK8963_register(AK8963_CNTL2,AK8963_RESET);
	// set gyro as clock source
	spi_write_register(spiImu, R_PWR_MGMT_1, CLOCK_SEL_PLL);	
	delay(100);
	// enable accel and gyro
	spi_write_register(spiImu, R_PWR_MGMT_2, SEN_ENABLE);	
	// set accel full scale
	spi_write_register(spiImu, R_ACCEL_CONFIG, ACCEL_FS_SEL_4G);
	// set gyro full scale
	spi_write_register(spiImu, R_GYRO_CONFIG, GYRO_FS_SEL_1000DPS);

	spi_write_register(spiImu, R_ACCEL_CONFIG2, ACCEL_DLPF_184);
	spi_write_register(spiImu, R_CONFIG, GYRO_DLPF_184);
	spi_write_register(spiImu, R_SMPLRT_DIV, 0x00);
   	spi_write_register(spiImu, R_USER_CTRL, I2C_MST_EN);
	spi_write_register(spiImu, R_I2C_MST_CTRL,I2C_MST_CLK);

	// power down magnetometer
	write_AK8963_register(AK8963_CNTL1, AK8963_PWR_DOWN);
	delay(100);
	// fuse rom access
	write_AK8963_register(AK8963_CNTL1, AK8963_FUSE_ROM);
	delay(100);
	read_AK8963_registers(AK8963_ASA, 3 , buffer);
	asaX = (int)buffer[0];
	asaY = (int)buffer[1];
	asaZ = (int)buffer[2];
	dbg_printf(("asaX = %d, asaY = %d, asaZ = %d\n", asaX, asaY, asaZ));	
	// power down magnetometer
	write_AK8963_register(AK8963_CNTL1, AK8963_PWR_DOWN);
	delay(100);
	// set 16bit resolution @ 100Hz
	write_AK8963_register(AK8963_CNTL1, AK8963_CNT_MEAS2);
	delay(100);
	spi_write_register(spiImu, R_PWR_MGMT_1, CLOCK_SEL_PLL);
	// accel + gyro odr = 500Hz, mag odr = 100Hz
	set_srd(1); 
	// interrupt is a 50us pulse
	spi_write_register(spiImu, R_INT_PIN_CFG, INT_PULSE_50US);
	// enable drdy interrupt
	spi_write_register(spiImu, R_INT_ENABLE, INT_RAW_RDY_EN);

	get_calib_params(Calib);
	}


void MPU9250::set_srd(uint8_t srd) {
	uint8_t buffer[7];
	// setting the sample rate divider to 19 to facilitate setting up magnetometer
	spi_write_register(spiImu, R_SMPLRT_DIV, 19);
	if(srd > 9){
		// set AK8963 to Power Down
		write_AK8963_register(AK8963_CNTL1, AK8963_PWR_DOWN);
		delay(100); // long wait between AK8963 mode changes  
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		write_AK8963_register(AK8963_CNTL1, AK8963_CNT_MEAS1);
		delay(100); // long wait between AK8963 mode changes     
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		read_AK8963_registers(AK8963_HXL, 7, buffer);
		} 
	else {
		// set AK8963 to Power Down
		write_AK8963_register(AK8963_CNTL1, AK8963_PWR_DOWN);
		delay(100); // long wait between AK8963 mode changes  
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		write_AK8963_register(AK8963_CNTL1, AK8963_CNT_MEAS2);
		delay(100); // long wait between AK8963 mode changes     
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		read_AK8963_registers(AK8963_HXL, 7, buffer);    
		} 
	// setting the sample rate divider 
	spi_write_register(spiImu, R_SMPLRT_DIV, srd);
	}


// place unit so that the sensor board accelerometer z axis is vertical. 
// This is where the sensor z axis sees a static  acceleration of 1g / -1g. 
// In this orientation the ax and ay values are the offsets for a 0g environment. 
// Repeat this calibration a few times with the debug serial monitor to check the 
// consistency of the calibration offsets. The board MUST be in a 1g static acceleration 
// environment for this calibration, i.e. at rest, no vibrations.

#define ACCEL_NUM_AVG_SAMPLES	100

void MPU9250::calibrate_accel(CALIB_PARAMS_t &calib){
	uint8_t buf[6];
	int16_t x,y,z;
	int32_t axAccum, ayAccum, azAccum;
	axAccum = ayAccum = azAccum = 0;
	for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
		spi_read_buffer(spiImu, R_ACCEL_XOUT_H | SPI_READ, 6, buf);
		x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
		y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
		z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
		axAccum += (int32_t) x;
		ayAccum += (int32_t) y;
		azAccum += (int32_t) z;
		ets_delay_us(2000); // 500Hz odr
		}
	x = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
	y = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
	z = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);
#ifdef MPU9250_DEBUG
	dbg_printf(("ax = %d  ay = %d  az = %d\r\n", x, y, z));
#endif

	calib.axBias = axBias = x;
	calib.ayBias = ayBias = y;
	calib.azBias = azBias = z > 0 ? z - (int16_t)(1000.0f*MPU9250_4G_SENSITIVITY) : z + (int16_t)(1000.0f*MPU9250_4G_SENSITIVITY);

#ifdef MPU9250_DEBUG
  dbg_printf(("axBias = %d\r\n", (int)axBias));
  dbg_printf(("ayBias = %d\r\n", (int)ayBias));
  dbg_printf(("azBias = %d\r\n", (int)azBias));
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
			spi_read_buffer(spiImu, R_GYRO_XOUT_H  | SPI_READ, 6, buf);
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
			ets_delay_us(2000); // 500Hz odr
			}
		} while (foundBadData && (++numTries < 10));

	// update gyro biases only if calibration succeeded, else use the last saved values from flash memory. Valid scenario for
	// gyro calibration failing is when you turn on the unit while flying. So not a big deal.
    if (!foundBadData) {		
		  calib.gxBias = gxBias =  (int16_t)( gxAccum / GYRO_NUM_CALIB_SAMPLES);
		  calib.gyBias = gyBias =  (int16_t)( gyAccum / GYRO_NUM_CALIB_SAMPLES);
		  calib.gzBias = gzBias =  (int16_t)( gzAccum / GYRO_NUM_CALIB_SAMPLES);		
		  }
#ifdef MPU9250_DEBUG
	dbg_printf(("Num Tries = %d\r\n",numTries));
	dbg_printf(("gxBias = %d\r\n",gxBias));
	dbg_printf(("gyBias = %d\r\n",gyBias));
	dbg_printf(("gzBias = %d\r\n",gzBias));
#endif
	return (foundBadData ? 0 : 1);
	}

#define MAG_NUM_CALIB_SAMPLES			4000
	
void MPU9250::calibrate_mag(CALIB_PARAMS_t &calib) {
	int16_t mx,my,mz;
	int mxs, mys, mzs, mxMin, myMin,mzMin,mxMax, myMax, mzMax;
	int mxSens, mySens, mzSens;
	mxMin = myMin = mzMin = 99999;
	mxMax = myMax = mzMax = -99999;

	for (int inx = 0; inx < MAG_NUM_CALIB_SAMPLES; inx++){
		get_vector(R_EXT_SENS_DATA_00, true, &mx, &my, &mz);
		mxs = (mx * (asaX+128))/256;
		mys = (my * (asaY+128))/256;
		mzs = (mz * (asaZ+128))/256;
		dbg_printf(("[%d] %d %d %d\n", inx, mxs, mys, mzs));
		if (mxs > mxMax) mxMax = mxs;
		if (mxs < mxMin) mxMin = mxs;
		if (mys > myMax) myMax = mys;
		if (mys < myMin) myMin = mys;
		if (mzs > mzMax) mzMax = mzs;
		if (mzs < mzMin) mzMin = mzs;
		delay(10); // 100Hz  odr
		}
	dbg_printf(("mxMin = %d, mxMax = %d",mxMin, mxMax));
	dbg_printf(("myMin = %d, myMax = %d",myMin, myMax));
	dbg_printf(("mzMin = %d, mzMax = %d",mzMin, mzMax));

	mxBias = (mxMin + mxMax)/2;
	myBias = (myMin + myMax)/2;
	mzBias = (mzMin + mzMax)/2;
	mxSens = (mxMax - mxMin)/2;
	mySens = (myMax - myMin)/2;
	mzSens = (mzMax - mzMin)/2;

	mxScale = 1000.0f/(float)mxSens;
	myScale = 1000.0f/(float)mySens;
	mzScale = 1000.0f/(float)mzSens;

	calib.mxBias = mxBias;
	calib.myBias = myBias;
	calib.mzBias = mzBias;
	calib.mxScale = mxScale;
	calib.myScale = myScale;
	calib.mzScale = mzScale;
	dbg_printf(("mxBias = %d, myBias = %d, mzBias = %d", mxBias, myBias, mzBias));
	dbg_printf(("mxScale  = %f, myScale = %f, mzScale = %f", mxScale, myScale, mzScale));
	}


void MPU9250::get_vector(uint8_t startAddr, int isLittleEndian, int16_t* px, int16_t* py, int16_t* pz) {
   uint8_t buf[6];
   spi_read_buffer(spiImu, startAddr | SPI_READ, 6, buf);
   if (isLittleEndian) {
		*px = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
		*py = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
		*pz = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);	
		}
   else {
		*px = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
		*py = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
		*pz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
		}
	}


void MPU9250::write_AK8963_register(uint8_t subAddress, uint8_t data){
	// set slave 0 to the AK8963 and set for write
	spi_write_register(spiImu, R_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
	// set the register to the desired AK8963 sub address 
	spi_write_register(spiImu, R_I2C_SLV0_REG, subAddress);
	// store the data for write
	spi_write_register(spiImu, R_I2C_SLV0_DO, data);
	// enable I2C and send 1 byte
	spi_write_register(spiImu, R_I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
	}


void MPU9250::read_AK8963_registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
	// set slave 0 to the AK8963 and set for read
	spi_write_register(spiImu, R_I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG);
	// set the register to the desired AK8963 sub address
	spi_write_register(spiImu, R_I2C_SLV0_REG, subAddress);
	// enable I2C and request the bytes
	spi_write_register(spiImu, R_I2C_SLV0_CTRL, I2C_SLV0_EN | count);
	ets_delay_us(1000); // takes some time for these registers to fill
	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	spi_read_buffer(spiImu, R_EXT_SENS_DATA_00 | SPI_READ, count, dest); 
	}
