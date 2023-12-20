// I2Cdev library collection - MS5611 I2C device class
// MS5611 library based on I2Cdev.h 2023, Pressure and Temperature measurement.
// STM 32 HAL library port : 03-11-2023 : BerkN
// Updates available at https://github.com/BerkN/MS5611
// Original I2Cdev library at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     03-11-2023 - Created for STM32 HAL library from MS5611 documentation.

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#include <stdint.h>
#include <stdbool.h>

#include "I2Cdev.h"

#ifndef MS5611_ADDRESS
#define MS5611_ADDRESS				0x77
#endif

#define MS5611_CMD_ADC_READ      	0x00
#define MS5611_CMD_RESET          	0x1E
#define MS5611_CMD_CONV_D1         	0x40
#define MS5611_CMD_CONV_D2        	0x50
#define MS5611_CMD_READ_PROM       	0xA0

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,		// 10 ms conversion time.
    MS5611_HIGH_RES         = 0x06,		// 5 ms conversion time.
    MS5611_STANDARD         = 0x04,		// 3 ms conversion time.
    MS5611_LOW_POWER        = 0x02,		// 2 ms conversion time.
    MS5611_ULTRA_LOW_POWER  = 0x00		// 1 ms conversion time.
} ms5611_osr_t;

/* Initialization routine */

bool MS5611_Init();
void MS5611_initConstants(bool mathMode);


void MS5611_setOversampling(ms5611_osr_t osr);
ms5611_osr_t MS5611_getOversampling();

uint32_t MS5611_getDeviceID();

/* Realtime application friendly functions:
 * MS5611 convertion takes time, you need to wait before read the adc.
 * The time between conversion & reading (ct : conversionTime ) is determined by the oversampling value.
 * When the oversampling value set, the conversionTime value is static. Pull for once and use over again.
 * conversionTime : in Milliseconds;
 * */

void MS5611_convert(const uint8_t addr);
void MS5611_getConvTime(uint8_t * ms_time);
void MS5611_adcRead(uint32_t* buffer);

void MS5611_processRawData(uint32_t D1 , uint32_t D2, bool compensation);
void MS5611_processTemperature(uint32_t D2 , bool compensation);

void MS5611_getPressure(int32_t* buffer);
void MS5611_getTemperature(int32_t* buffer);

void MS5611_setPressureOffset(int32_t offset);
void MS5611_setTemperatureOffset(int32_t offset);

/* Functions below, not usable for real time applications because of HAL_Delay.
 * MS5611_read() routine is example for how to access pressure and temperature data.
 * */

/* Sends conversion command to the MS5611 and waits for adc conversion.
 * After reading the raw pressure and temperature values (_D1 & _D2),
 * MS5611_processRawData(_D1,_D2,true) function is called.
 */
void MS5611_read();

/* Read the Raw pressure from MS5611
 * Warning : This function waits fo the conversiontime (ct) (1-10ms)
 * */
uint32_t MS5611_readRawPressure();

/* Read the Raw temperature from MS5611
 * Warning : This function waits fo the conversiontime (ct) (1-10ms)
 * */
uint32_t MS5611_readRawTemperature();

#endif /* INC_MS5611_H_ */
