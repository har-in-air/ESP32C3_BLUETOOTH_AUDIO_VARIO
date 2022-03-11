#include <Arduino.h>
#include "config.h"
#include "spi.h"
#include "ms5611.h"

MS5611::MS5611() {
	pressurePa = 0.0f;
	altitudeCm = 0.0f;
	altitudeCmAvg = 0.0f;
	temperatureC = 0;
	}

#if 0	

#define MAX_TEST_SAMPLES    100
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611::measure_noise(int nSamples) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
	n = 5;
	//  throwaway samples
    while (n--) {
    	trigger_temperature_sample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2 = read_sample();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1 = read_sample();
		}
    for (n = 0; n < nSamples; n++) {
	    trigger_temperature_sample();
	    delay(MS5611_SAMPLE_PERIOD_MS);
	    D2 = read_sample();
	    calculate_temperatureCx10();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1 = read_sample();
		pa[n] = calculate_pressurePa();
        z[n] =  pa_to_cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    dbg_printf(("paMean = %dPa, zMean = %dcm\n",(int)paMean,(int)zMean));
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        //dbg_printf(("%d %d\r\n",(int)pa[n],(int)z[n]));
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    dbg_printf(("\npaVariance %d  zVariance %d\n",(int)paVariance, (int)zVariance));
	}
#endif

void MS5611::averaged_sample(int nSamples) {
	int32_t tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = 5;
	//  throwaway samples
    while (n--) {
    	trigger_temperature_sample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2 = read_sample();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1 = read_sample();
		}
	
	n = nSamples;
    while (n--) {
    	trigger_temperature_sample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2 = read_sample();
		calculate_temperatureCx10();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1 = read_sample();
		pa = calculate_pressurePa();
		pAccum += pa;
		tAccum += tempCx100;
		}
	tc = tAccum/nSamples;
	temperatureC = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	pressurePa = (pAccum+nSamples/2)/nSamples;
	altitudeCmAvg = altitudeCm = pa_to_cm(pressurePa);
#ifdef MS5611_DEBUG
   dbg_printf(("Tavg : %dC\r\n", temperatureC));
   dbg_printf(("Pavg : %dPa\r\n",(int)pressurePa));
   dbg_printf(("Zavg : %dcm\r\n",(int)altitudeCmAvg));
#endif
	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.h"

float MS5611::pa_to_cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)(paf);

   	if (pa > PA_INIT) {
      	zf = (float)(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = (float)(z1) + ( ((float)pa1-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}

void MS5611::calculate_temperatureCx10(void) {
	dT = (int64_t)D2 - tref;
	tempCx100 = 2000 + ((dT*((int32_t)cal[5]))>>23);
	}


float MS5611::calculate_pressurePa(void) {
	float pa;
    int64_t offset, sens,offset2,sens2,t2;
	offset = offT1 + ((((int64_t)cal[3])*dT)>>7);
	sens = sensT1 + ((((int64_t)cal[2])*dT)>>8);
    if (tempCx100 < 2000) { // correction for temperature < 20C
        t2 = ((dT*dT)>>31); 
        offset2 = (5*(tempCx100-2000)*(tempCx100-2000))/2;
        sens2 = offset2/2;
        } 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100 -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = (((float)((int64_t)D1 * sens))/2097152.0f - (float)offset) / 32768.0f;
	return pa;
	}


/// Trigger a pressure sample with max oversampling rate
void MS5611::trigger_pressure_sample(void) {
	spi_write_command(spiBaro, MS5611_CONVERT_D1 | MS5611_ADC_4096);
	}

/// Trigger a temperature sample with max oversampling rate
void MS5611::trigger_temperature_sample(void) {
	spi_write_command(spiBaro, MS5611_CONVERT_D2 | MS5611_ADC_4096);
	}


uint32_t MS5611::read_sample(void)	{
	uint8_t buf[3];
	spi_read_buffer(spiBaro, MS5611_ADC_READ, 3, buf);
	uint32_t w = (((uint32_t)buf[0])<<16) | (((uint32_t)buf[1])<<8) | (uint32_t)buf[2];
	return w;
   }


void MS5611::init_sample_state_machine(void) {
   trigger_temperature_sample();
   sensorState = MS5611_READ_TEMPERATURE;
   }

int MS5611::sample_state_machine(void) {
   if (sensorState == MS5611_READ_TEMPERATURE) {
      D2 = read_sample();
      trigger_pressure_sample();
      //DBG_1(); // turn on the debug pulse for timing the critical computation
      calculate_temperatureCx10();
      //celsiusSample_ = (tempCx100 >= 0? (tempCx100+50)/100 : (tempCx100-50)/100);
      pressurePa = calculate_pressurePa();
	  altitudeCm = pa_to_cm(pressurePa);
      //DBG_0();
	  sensorState = MS5611_READ_PRESSURE;
      return 1;  // 1 => new altitude sample is available
      }
   else
   if (sensorState == MS5611_READ_PRESSURE) {
      D1 = read_sample();
      trigger_temperature_sample();
      sensorState = MS5611_READ_TEMPERATURE;
      return 0; // 0 => intermediate state
      }
   return 0;    
   }


void MS5611::reset() {
	spi_write_command(spiBaro, MS5611_RESET);
	ets_delay_us(3000); // 3mS as per app note AN520	
    }
   
	
void MS5611::get_calib_coefficients(void)  {
	// PROM[0] = reserved, PROM[7] = CRC
    for (int inx = 0; inx < 6; inx++) {
		cal[inx] = prom[inx+1];
		}
#ifdef MS5611_DEBUG
    dbg_printf(("MS5611 Calibration Coeffs : %d %d %d %d %d %d\r\n",cal[0],cal[1],cal[2],cal[3],cal[4],cal[5]));
#endif	 
    tref = ((int64_t)cal[4])<<8;
    offT1 = ((int64_t)cal[1])<<16;
    sensT1 = ((int64_t)cal[0])<<15;		
    }
   
int MS5611::read_prom(void)    {
	uint8_t buf[2];
    for (int inx = 0; inx < 8; inx++) {
		spi_read_buffer(spiBaro, 0xA0 + inx*2, 2, buf);
		prom[inx] = ((uint16_t)buf[0])<<8 | (uint16_t)buf[1];
		}			
	//dbg_printf(("\r\nProm : "));
	//for (int inx = 0; inx < 16; inx++) {
	//	dbg_printf(("0x%02x ", prom_[inx]));
	//	}
	//dbg_println(());
	uint8_t crcPROM = (uint8_t)(prom[7] & 0x000F);
	uint8_t crcCalculated = crc4(prom);
	dbg_printf(("crcPROM = 0x%X, crc calculated = 0x%X\n", crcPROM, crcCalculated));
	return (crcCalculated == crcPROM ? 1 : 0);
	}
	

// test: 0xB for {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};	
uint8_t MS5611::crc4(uint16_t* prom ) {
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;
	n_rem = 0x0000;
	crc_read = prom[7];
	prom[7] &= 0xFF00;
	for (int cnt = 0; cnt < 16; cnt++){
		if (cnt%2 == 1) {
			n_rem ^= (uint16_t)((prom[cnt>>1]) & 0x00FF);
			}
		else {
			n_rem ^= (uint16_t) (prom[cnt>>1] >> 8);
			}
		for (int nbit = 8; nbit > 0; nbit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
				}
			else {
				n_rem = n_rem << 1;
				}
			}
		}
	n_rem = (0x000F & (n_rem>>12)); // final 4bit remainder is CRC code
	prom[7] = crc_read; // restore crc_read to its original place;
	return (n_rem ^ 0x00);
	}
	
