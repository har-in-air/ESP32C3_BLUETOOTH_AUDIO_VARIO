#include <Arduino.h>
#include "cct.h"

static const char* TAG = "cct";

static uint32_t CCT_TICKS_PER_US;

void cct_init() {
	CCT_TICKS_PER_US = getCpuFrequencyMhz();
	}

void cct_delayUs(uint32_t us) {
	volatile uint32_t waitCycleCount = (CCT_TICKS_PER_US * us ) + cpu_hal_get_cycle_count();
	do  {} while (cpu_hal_get_cycle_count()  < waitCycleCount);
	}


uint32_t  cct_get_intervalUs(uint32_t before, uint32_t after) {
	return  (before <= after ?
		((after - before)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
		(after + (0xFFFFFFFF - before) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
	}


float  cct_get_intervalSecs(uint32_t before, uint32_t after) {
	return  (before <= after ?
		(float)(after - before)/(float)(CCT_TICKS_PER_US*1000000) :
		(float)(after + (0xFFFFFFFF - before))/(float)(CCT_TICKS_PER_US*1000000));
	}


uint32_t cct_get_elapsedUs(uint32_t clockPrev) {
	uint32_t clockNow = cpu_hal_get_cycle_count();
	return  (clockPrev <= clockNow ?
		((clockNow - clockPrev) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
		(clockNow + (0xFFFFFFFF - clockPrev) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
	}

