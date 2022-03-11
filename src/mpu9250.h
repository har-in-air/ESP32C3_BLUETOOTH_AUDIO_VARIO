#ifndef _MPU9250_H_
#define _MPU9250_H_

#include "nvd.h"

#define R_SMPLRT_DIV	0x19
#define R_CONFIG		0x1A
	#define GYRO_DLPF_184	0x01
	#define GYRO_DLPF_92	0x02
	#define GYRO_DLPF_41	0x03
	#define GYRO_DLPF_20	0x04
	#define GYRO_DLPF_10	0x05
	#define GYRO_DLPF_5		0x06

#define R_GYRO_CONFIG		0x1B
	#define GYRO_FS_SEL_250DPS	0x00
	#define GYRO_FS_SEL_500DPS	0x08
	#define GYRO_FS_SEL_1000DPS	0x10
	#define GYRO_FS_SEL_2000DPS	0x18

#define R_ACCEL_CONFIG		0x1C
	#define ACCEL_FS_SEL_2G		0x00
	#define ACCEL_FS_SEL_4G		0x08
	#define ACCEL_FS_SEL_8G		0x10
	#define ACCEL_FS_SEL_16G	0x18

#define R_ACCEL_CONFIG2		0x1D
	#define ACCEL_DLPF_184	0x01
	#define ACCEL_DLPF_92	0x02
	#define ACCEL_DLPF_41	0x03
	#define ACCEL_DLPF_20	0x04
	#define ACCEL_DLPF_10	0x05
	#define ACCEL_DLPF_5	0x06

#define R_LP_ACCEL_ODR		0x1E

#define R_FIFO_EN			0x23
#define R_INT_PIN_CFG		0x37
#define R_INT_ENABLE		0x38
	#define INT_DISABLE		0x00
	#define INT_PULSE_50US	0x00
	#define INT_RAW_RDY_EN	0x01
#define R_INT_STATUS		0x3A

#define R_ACCEL_XOUT_H		0x3B
#define R_TEMP_OUT_H		0x41
#define R_GYRO_XOUT_H		0x43
#define R_EXT_SENS_DATA_00	0x49

#define R_USER_CTRL			0x6A
	#define I2C_MST_EN  0x20
	#define I2C_IF_DIS  0x10

#define R_PWR_MGMT_1		0x6B // Device defaults to the SLEEP mode
	#define PWR_CYCLE		0x20
	#define PWR_RESET		0x80
	#define CLOCK_SEL_PLL	0x01

#define R_PWR_MGMT_2		0x6C
	#define SEN_ENABLE	0x00
	#define DIS_GYRO	0x07

#define R_I2C_MST_CTRL		0x24
	#define I2C_MST_CLK 0x0D

#define R_I2C_SLV0_ADDR		0x25
#define R_I2C_SLV0_REG		0x26
#define R_I2C_SLV0_CTRL		0x27
#define R_I2C_SLV0_DO		0x63
	#define I2C_SLV0_EN			0x80
	#define I2C_READ_FLAG		0x80
	#define AK8963_I2C_ADDR		0x0C
	#define AK8963_HXL			0x03 
	#define AK8963_CNTL1		0x0A
	#define AK8963_PWR_DOWN		0x00
	#define AK8963_CNT_MEAS1	0x12
	#define AK8963_CNT_MEAS2	0x16
	#define AK8963_FUSE_ROM		0x0F
	#define AK8963_CNTL2		0x0B
	#define AK8963_RESET		0x01
	#define AK8963_ASA			0x10
	#define AK8963_WHO_AM_I		0x00

#define R_WHO_AM_I		0x75 // Should return 0x71

#define MPU9250_2G_SENSITIVITY		16.384f 	// lsb per milli-g
#define MPU9250_4G_SENSITIVITY		8.192f   // lsb per milli-g

#define MPU9250_500DPS_SENSITIVITY	65.5f 		// lsb per deg/sec
#define MPU9250_1000DPS_SENSITIVITY	32.8f     // lsb per deg/sec

#define SPI_READ  0x80


class MPU9250 {	
  public:
    MPU9250();
	int check_id(void);
	void get_accel_gyro_data(float* pAccelData, float* pGyroData);
	void get_accel_gyro_mag_data(float* pAccelData, float* pGyroData, float* pMagData);
    void config_accel_gyro(void);
	void config_accel_gyro_mag(void);
    int  calibrate_gyro(CALIB_PARAMS_t &calib);
    void calibrate_accel(CALIB_PARAMS_t &calib);
    void calibrate_mag(CALIB_PARAMS_t &calib);
	void get_calib_params(CALIB_PARAMS_t &calib);
	void sleep(void);

private :
    int16_t axBias;
    int16_t ayBias;
    int16_t azBias;
	int16_t gxBias;
	int16_t gyBias;
	int16_t gzBias;	
	int16_t mxBias;
	int16_t myBias;
	int16_t mzBias;
	int asaX, asaY, asaZ;
	float mxScale;
	float myScale;
	float mzScale;
	float aScale;
	float gScale;
	void set_srd(uint8_t srd);
	void write_AK8963_register(uint8_t subAddress, uint8_t data);	
	void read_AK8963_registers(uint8_t subAddress, uint8_t count, uint8_t* dest);	
	void get_vector(uint8_t startAddr, int isLittleEndian, int16_t* px, int16_t* py, int16_t* pz);

	};  

#endif // _MPU9250_H_
