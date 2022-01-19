#ifndef CCT_H_
#define CCT_H_

#define cct_setMarker()     cpu_hal_get_cycle_count()

void cct_init();
void cct_delayUs(uint32_t us);
uint32_t  cct_get_intervalUs(uint32_t before, uint32_t after);
uint32_t cct_get_elapsedUs(uint32_t clockPrev);
float  cct_get_intervalSecs(uint32_t before, uint32_t after);


#endif