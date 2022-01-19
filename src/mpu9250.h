#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "nvd.h"

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E

#define FIFO_EN            0x23
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48

#define MOT_DETECT_STATUS  0x61
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define PWR_MGMT_1         0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define WHO_AM_I_MPU9250   0x75 // Should return 0x71

// i2c 7bit device address is 110 100[AD0] = 0x68
#define MPU9250_I2C_ADDRESS 	0x68  

#define MPU9250_2G_SENSITIVITY 		16.384f 	// lsb per milli-g
#define MPU9250_4G_SENSITIVITY     8.192f   // lsb per milli-g

#define MPU9250_500DPS_SENSITIVITY	65.5f 		// lsb per deg/sec
#define MPU9250_1000DPS_SENSITIVITY  32.8f     // lsb per deg/sec


class MPU9250 {
private :
	float aScale_;
	float gScale_;
	
  public:
    MPU9250();
	int check_id(void);
	void get_accel_gyro_data(float* pAccelData, float* pGyroData);
    void config_accel_gyro(void);
    int  calibrate_gyro(CALIB_PARAMS_t &calib);
    void calibrate_accel(CALIB_PARAMS_t &calib);
	void get_calib_params(CALIB_PARAMS_t &calib);
	void sleep(void);
    void write_byte(uint8_t deviceID, uint8_t addr, uint8_t val);
    uint8_t read_byte(uint8_t deviceID, uint8_t addr);
    int read_bytes(uint8_t deviceID, uint8_t addr, uint8_t numBytes, uint8_t *pBuf);

    int16_t axBias_;
    int16_t ayBias_;
    int16_t azBias_;
	int16_t gxBias_;
	int16_t gyBias_;
	int16_t gzBias_;	
	};  

#endif // _MPU9250_H_
