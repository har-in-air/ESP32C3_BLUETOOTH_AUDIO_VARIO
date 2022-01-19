#ifndef NVD_H_
#define NVD_H_

#include "config.h"

typedef struct  {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	} CALIB_PARAMS_t;	

typedef struct  {
	int16_t  climbThresholdCps;
	int16_t  zeroThresholdCps;
	int16_t  sinkThresholdCps;
	int16_t  crossoverCps;
	} VARIO_PARAMS_t;

typedef struct  {
	int16_t  accelVariance; // environmental acceleration disturbance variance, divided by 1000
	int16_t  zMeasVariance; // z measurement noise variance
	} KALMAN_FILTER_PARAMS_t;

typedef struct  {
	int16_t  sleepTimeoutMinutes;
	int16_t  bleEnable;
	} MISC_PARAMS_t;

typedef struct {
	String ssid;
	String password;
} AP_CRED_t;

typedef struct  {
	AP_CRED_t cred;
	VARIO_PARAMS_t vario;
	KALMAN_FILTER_PARAMS_t kf;
	MISC_PARAMS_t misc;
	} CONFIG_PARAMS_t;

extern CONFIG_PARAMS_t Config;
extern CALIB_PARAMS_t Calib;

void nvd_calib_load(CALIB_PARAMS_t &calib);
void nvd_calib_reset(CALIB_PARAMS_t &calib);
void nvd_calib_store(CALIB_PARAMS_t &calib);

void nvd_config_load(CONFIG_PARAMS_t &config);
void nvd_config_reset(CONFIG_PARAMS_t &config);
void nvd_config_store(CONFIG_PARAMS_t &config);


#endif
