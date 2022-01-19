#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "nvd.h"
#include "util.h"

#define MODE_READ_WRITE  false
#define MODE_READ_ONLY   true

// saves and retrieves non-volatile data (NVD) to / from ESP32-C3 flash. 
Preferences Prefs;

CALIB_PARAMS_t Calib;
CONFIG_PARAMS_t Config;

void nvd_calib_load(CALIB_PARAMS_t &calib) {
	if (Prefs.begin("calib", MODE_READ_ONLY) == false) {
		dbg_println(("Preferences 'calib' namespace not found, creating with defaults"));
		Prefs.end();
		nvd_calib_reset(calib);
		}
	else {
		calib.axBias = Prefs.getInt("axBias", 0); // set to 0 if not found 
		calib.ayBias = Prefs.getInt("ayBias", 0);
		calib.azBias = Prefs.getInt("azBias", 0);
		calib.gxBias = Prefs.getInt("gxBias", 0);
		calib.gyBias = Prefs.getInt("gyBias", 0);
		calib.gzBias = Prefs.getInt("gzBias", 0);
		Prefs.end();
#ifdef NVD_DEBUG	
		dbg_println(("ACCEL & GYRO Calibration Values"));
		dbg_printf(("axBias = %d\r\n", calib.axBias));
		dbg_printf(("ayBias = %d\r\n", calib.ayBias));
		dbg_printf(("azBias = %d\r\n", calib.azBias));
		dbg_printf(("gxBias = %d\r\n", calib.gxBias));
		dbg_printf(("gyBias = %d\r\n", calib.gyBias));
		dbg_printf(("gzBias = %d\r\n", calib.gzBias));
#endif		
		}
	}


void nvd_calib_reset(CALIB_PARAMS_t &calib) {
	calib.axBias = 0;
	calib.ayBias = 0;
	calib.azBias = 0;
	calib.gxBias = 0;
	calib.gyBias = 0;
	calib.gzBias = 0;
	nvd_calib_store(calib);
	}	


void nvd_calib_store(CALIB_PARAMS_t &calib) {
	Prefs.begin("calib", MODE_READ_WRITE);
	Prefs.clear();
	Prefs.putInt("axBias", calib.axBias);
	Prefs.putInt("ayBias", calib.ayBias);
	Prefs.putInt("azBias", calib.azBias);
	Prefs.putInt("gxBias", calib.gxBias);
	Prefs.putInt("gyBias", calib.gyBias);
	Prefs.putInt("gzBias", calib.gzBias);
	Prefs.end();
	}


void nvd_config_load(CONFIG_PARAMS_t &config) {
	if (Prefs.begin("config", MODE_READ_ONLY) == false) {
		dbg_println(("Preferences 'config' namespace not found, creating with defaults"));
		Prefs.end();
		nvd_config_reset(config);
		}
	else {
		config.cred.ssid = Prefs.getString("ssid", "");
		config.cred.password = Prefs.getString("password", "");

		config.vario.climbThresholdCps = Prefs.getInt("climbTh", VARIO_CLIMB_THRESHOLD_CPS_DEFAULT);
		config.vario.zeroThresholdCps = Prefs.getInt("zeroTh", VARIO_ZERO_THRESHOLD_CPS_DEFAULT);
		config.vario.sinkThresholdCps = Prefs.getInt("sinkTh", VARIO_SINK_THRESHOLD_CPS_DEFAULT);
		config.vario.crossoverCps = Prefs.getInt("xoverTh", VARIO_CROSSOVER_CPS_DEFAULT);

		config.kf.accelVariance = Prefs.getInt("accVar", KF_ACCEL_VARIANCE_DEFAULT);
		config.kf.zMeasVariance = Prefs.getInt("zmeasVar", KF_ZMEAS_VARIANCE_DEFAULT);

		config.misc.bleEnable = Prefs.getBool("bleEna", BLE_DEFAULT);
		config.misc.sleepTimeoutMinutes = Prefs.getInt("sleepTime", SLEEP_TIMEOUT_MINUTES_DEFAULT);
		Prefs.end();
#ifdef NVD_DEBUG	
		dbg_println(("WiFi AP credentials"));
		dbg_printf(("SSID = %s\r\n", config.cred.ssid));

		dbg_println(("VARIO"));
		dbg_printf(("climbThresholdCps = %d\r\n", config.vario.climbThresholdCps));
		dbg_printf(("zeroThresholdCps = %d\r\n", config.vario.zeroThresholdCps));
		dbg_printf(("sinkThresholdCps = %d\r\n", config.vario.sinkThresholdCps));
		dbg_printf(("crossoverCps = %d\r\n", config.vario.crossoverCps));
		
		dbg_println(("KALMAN FILTER"));
		dbg_printf(("accelVariance = %d\r\n", config.kf.accelVariance));
		dbg_printf(("zMeasVariance = %d\r\n", config.kf.zMeasVariance));
			
		dbg_println(("MISCELLANEOUS"));
		dbg_printf(("sleepTimeoutMinutes = %d\r\n", config.misc.sleepTimeoutMinutes));
		dbg_printf(("bleEnable = %s\r\n", config.misc.bleEnable ? "true" : "false"));
#endif
		}
	}


void nvd_config_reset(CONFIG_PARAMS_t &config) {
	config.cred.ssid = "";
	config.cred.password = "";

	config.vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
	config.vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
	config.vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT;
	config.vario.crossoverCps = VARIO_CROSSOVER_CPS_DEFAULT;

	config.kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
	config.kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

	config.misc.bleEnable = BLE_DEFAULT;
	config.misc.sleepTimeoutMinutes = SLEEP_TIMEOUT_MINUTES_DEFAULT;
	nvd_config_store(config);
	}


void nvd_config_store(CONFIG_PARAMS_t &config) {
	Prefs.begin("config", MODE_READ_WRITE);
	Prefs.clear();
	Prefs.putString("ssid", config.cred.ssid);
	Prefs.putString("password", config.cred.password);

	Prefs.putInt("climbTh", config.vario.climbThresholdCps);
	Prefs.putInt("zeroTh", config.vario.zeroThresholdCps);
	Prefs.putInt("sinkTh", config.vario.sinkThresholdCps);
	Prefs.putInt("xoverTh", config.vario.crossoverCps);

	Prefs.putInt("accVar", config.kf.accelVariance);
	Prefs.putInt("zmeasVar", config.kf.zMeasVariance);

	Prefs.putBool("bleEna", config.misc.bleEnable);
	Prefs.putInt("sleepTime", config.misc.sleepTimeoutMinutes);
	Prefs.end();
	}	

