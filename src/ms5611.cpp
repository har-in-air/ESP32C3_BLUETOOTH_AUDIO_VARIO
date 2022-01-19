#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "ms5611.h"

MS5611::MS5611() {
	pressurePa = 0.0f;
	altitudeCm = 0.0f;
	altitudeCmAvg = 0.0f;
	temperatureC = 0;
	}

#if 0	

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611::test(int nSamples) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    trigger_temperature_sample();
	    delay(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = read_sample();
	    calculate_temperatureCx10();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa[n] = calculate_pressurePa();
        z[n] =  pa_to_cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    dbg_printf("paMean = %dPa, zMean = %dcm\r\n",(int)paMean,(int)zMean);
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        //dbg_printf("%d %d\r\n",(int)pa[n],(int)z[n]);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    dbg_printf("\r\npaVariance %d  zVariance %d\r\n",(int)paVariance, (int)zVariance);
	}
#endif

void MS5611::averaged_sample(int nSamples) {
	int32_t tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = 2;
    while (n--) {
    	trigger_temperature_sample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = read_sample();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = read_sample();
		}
	
	n = nSamples;
    while (n--) {
    	trigger_temperature_sample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = read_sample();
		calculate_temperatureCx10();
		trigger_pressure_sample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = read_sample();
		pa = calculate_pressurePa();
		pAccum += pa;
		tAccum += tempCx100_;
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
	dT_ = (int64_t)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((int32_t)cal_[5]))>>23);
	}


float MS5611::calculate_pressurePa(void) {
	float pa;
    int64_t offset, sens,offset2,sens2,t2;
	offset = offT1_ + ((((int64_t)cal_[3])*dT_)>>7);
	sens = sensT1_ + ((((int64_t)cal_[2])*dT_)>>8);
    if (tempCx100_ < 2000) { // correction for temperature < 20C
        t2 = ((dT_*dT_)>>31); 
        offset2 = (5*(tempCx100_-2000)*(tempCx100_-2000))/2;
        sens2 = offset2/2;
        } 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = (((float)((int64_t)D1_ * sens))/2097152.0f - (float)offset) / 32768.0f;
	return pa;
	}


/// Trigger a pressure sample with max oversampling rate
void MS5611::trigger_pressure_sample(void) {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_CONVERT_D1 | MS5611_ADC_4096);  //  pressure conversion, max oversampling
	Wire.endTransmission();  
   }

/// Trigger a temperature sample with max oversampling rate
void MS5611::trigger_temperature_sample(void) {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_CONVERT_D2 | MS5611_ADC_4096);   //  temperature conversion, max oversampling
	Wire.endTransmission();  
   }

uint32_t MS5611::read_sample(void)	{
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  // Initialize the Tx buffer
	Wire.write(0x00);                        // Put ADC read command in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(MS5611_I2C_ADDRESS, 3);     // Read three bytes from slave PROM address 
	int inx = 0;
	uint8_t buf[3];
	while (Wire.available()) {
        buf[inx++] = Wire.read(); 
		}          
	uint32_t w = (((uint32_t)buf[0])<<16) | (((uint32_t)buf[1])<<8) | (uint32_t)buf[2];
	return w;
   }


void MS5611::init_sample_state_machine(void) {
   trigger_temperature_sample();
   sensorState = MS5611_READ_TEMPERATURE;
   }

int MS5611::sample_state_machine(void) {
   if (sensorState == MS5611_READ_TEMPERATURE) {
      D2_ = read_sample();
      trigger_pressure_sample();
      //DBG_1(); // turn on the debug pulse for timing the critical computation
      calculate_temperatureCx10();
      //celsiusSample_ = (tempCx100_ >= 0? (tempCx100_+50)/100 : (tempCx100_-50)/100);
      pressurePa = calculate_pressurePa();
	  altitudeCm = pa_to_cm(pressurePa);
      //DBG_0();
	  sensorState = MS5611_READ_PRESSURE;
      return 1;  // 1 => new altitude sample is available
      }
   else
   if (sensorState == MS5611_READ_PRESSURE) {
      D1_ = read_sample();
      trigger_temperature_sample();
      sensorState = MS5611_READ_TEMPERATURE;
      return 0; // 0 => intermediate state
      }
   return 0;    
   }

void MS5611::reset() {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_RESET);                
	Wire.endTransmission();       
	delay(10); // 3mS as per app note AN520	
    }
   
	
void MS5611::get_calib_coefficients(void)  {
    for (int inx = 0; inx < 6; inx++) {
		int promIndex = 2 + inx*2; 
		cal_[inx] = (((uint16_t)prom_[promIndex])<<8) | (uint16_t)prom_[promIndex+1];
		}
#ifdef MS5611_DEBUG
    dbg_printf(("MS5611 Calibration Coeffs : %d %d %d %d %d %d\r\n",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]));
#endif	 
    tref_ = ((int64_t)cal_[4])<<8;
    offT1_ = ((int64_t)cal_[1])<<16;
    sensT1_ = ((int64_t)cal_[0])<<15;		
    }
   
int MS5611::read_prom(void)    {
    for (int inx = 0; inx < 8; inx++) {
		Wire.beginTransmission(MS5611_I2C_ADDRESS); 
		Wire.write(0xA0 + inx*2); 
		Wire.endTransmission(false); // restart
		Wire.requestFrom(MS5611_I2C_ADDRESS, 2); 
		int cnt = 0;
		while (Wire.available()) {
			prom_[inx*2 + cnt] = Wire.read(); 
			cnt++;
			}
		}			
	//dbg_printf(("\r\nProm : "));
	//for (int inx = 0; inx < 16; inx++) {
	//	dbg_printf(("0x%02x ", prom_[inx]));
	//	}
	//dbg_println(());
	uint8_t crcPROM = prom_[15] & 0x0F;
	uint8_t crcCalculated = crc4(prom_);
	return (crcCalculated == crcPROM ? 1 : 0);
	}
	
	
uint8_t MS5611::crc4(uint8_t prom[] ) {
	 int cnt, nbit; 
	 uint16_t crcRemainder; 
	 uint8_t crcSave = prom[15]; // crc byte in PROM
#ifdef MS5611_DEBUG
	 dbg_printf(("MS5611 PROM CRC = 0x%x\r\n", prom[15] & 0x0F));
#endif	 
	 crcRemainder = 0x0000;
	 prom[15] = 0; //CRC byte is replaced by 0
	 
	 for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000; 
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
	 crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
	 prom[15] = crcSave; // restore the crc byte
#ifdef MS5611_DEBUG
	 dbg_printf(("Calculated CRC = 0x%x\r\n",  crcRemainder ^ 0x0));
#endif	 
	 return (uint8_t)(crcRemainder ^ 0x0);
	} 
	
