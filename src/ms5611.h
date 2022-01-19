#ifndef MS5611_H_
#define MS5611_H_

//#define MS5611_TEST

// max conversion time with OSR=4096 is  9.04mS
#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22


#define MS5611_I2C_ADDRESS 0x77

#define MS5611_RESET      	0x1E
#define MS5611_CONVERT_D1 	0x40
#define MS5611_CONVERT_D2 	0x50
#define MS5611_ADC_READ   	0x00

#define MS5611_ADC_4096 	0x08

class MS5611 {        
public :
    MS5611();
    void trigger_pressure_sample(void);
    void trigger_temperature_sample(void);
    uint32_t  read_sample(void);
    void averaged_sample(int nSamples);
    void calculate_temperatureCx10(void);
    float calculate_pressurePa(void);
    void calculate_sensor_noisePa(void);
    void reset(void);

    void get_calib_coefficients(void);
    float pa_to_cm(float pa);
    void test(int nSamples);
    int  read_prom(void);
    uint8_t crc4(uint8_t prom[] );
    int  sample_state_machine(void);
    void init_sample_state_machine(void);

    volatile float pressurePa;
    volatile float altitudeCm;
    float altitudeCmAvg;
    int  temperatureC;
    volatile int sensorState;

private :
    uint8_t prom_[16];
    uint16_t cal_[6];
    int64_t tref_;
    int64_t offT1_;
    int64_t sensT1_;
    int32_t tempCx100_;
    uint32_t D1_;
    uint32_t D2_;
    int64_t dT_;
    };

#endif // MS5611_H_
