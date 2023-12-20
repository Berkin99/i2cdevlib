// I2Cdev library collection - MS5611 I2C device class
// MS5611 library based on I2Cdev.h 2023, Pressure and Temperature measurement.
// STM 32 HAL library port : 03-11-2023 : BerkN
// Updates available at https://github.com/BerkN/MS5611
// Original I2Cdev library at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     03-11-2023 - Created for STM32 HAL library from MS5611 documentation.

#include "MS5611.h"

uint32_t _deviceID;
uint8_t _samplingRate;
uint8_t ct;

/*Stored PROM values multiplied by calculation constants for optimisation*/
float _C[7];

/* _D1 : Digital value 1 : Raw pressure value readed from adc of MS5611.
 * _D2 : Digital value 2 : Raw temperature value readed from adc of MS5611.
 * _temperature : Processed temperature value as integer : 2507 means 25.07 Celcius
 * _pressure : Processed pressure value as integer : 99876 means 998.76 mbar.
 */
uint32_t _D1, _D2;

int32_t  _pressure;
int32_t  _temperature;
int32_t _pressureOffset;
int32_t _temperatureOffset;

void reset();
uint16_t readPROM(uint8_t reg);



bool MS5611_Init(){
    reset();
    MS5611_setOversampling(MS5611_HIGH_RES);
    MS5611_initConstants(false);
    HAL_Delay(10);

    _deviceID = 0;
    bool PROM = true;
    for (uint8_t reg = 0; reg < 7; reg++)
    {
      uint16_t tmp = readPROM(reg);
      _C[reg] *= tmp;

      _deviceID <<= 4;
      _deviceID ^= tmp;

      if ((reg > 0)&&(tmp == 0)) PROM = false;
    }

    return PROM;
}

void MS5611_setOversampling(ms5611_osr_t osr){

	switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

	_samplingRate = (uint8_t) osr;

}

ms5611_osr_t MS5611_getOversampling(){
	return (ms5611_osr_t)_samplingRate;
}

void MS5611_initConstants(bool mathMode)
{

	_C[0] = 1;
	_C[1] = 32768L;          	//  SENSt1   = C[1] * 2^15    |    * 2^16
	_C[2] = 65536L;          	//  OFFt1    = C[2] * 2^16    |    * 2^17
	_C[3] = 3.90625E-3;      	//  TCS      = C[3] / 2^8     |    / 2^7
	_C[4] = 7.8125E-3;       	//  TCO      = C[4] / 2^7     |    / 2^6
	_C[5] = 256;             	//  Tref     = C[5] * 2^8     |    * 2^8
	_C[6] = 1.1920928955E-7; 	//  TEMPSENS = C[6] / 2^23    |    / 2^23

	if (mathMode)
	{
		_C[1] = 65536L;         //  SENSt1
		_C[2] = 131072L;        //  OFFt1
		_C[3] = 7.8125E-3;      //  TCS
		_C[4] = 1.5625e-2;      //  TCO
	}
}


void reset()
{
	I2Cdev_command(MS5611_ADDRESS, MS5611_CMD_RESET);
	_pressureOffset = 0;
	_temperatureOffset = 0;
}

uint16_t readPROM(uint8_t reg)
{
	uint8_t tmp[2] = {0,0};
	I2Cdev_readBytes(MS5611_ADDRESS,(MS5611_CMD_READ_PROM + (reg * 2)),2,tmp,1000);
    return tmp[0]<<8 | tmp[1];
}

uint32_t MS5611_getDeviceID() { return _deviceID; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MS5611_read(){
	MS5611_convert(MS5611_CMD_CONV_D1);
	HAL_Delay(ct);
	MS5611_adcRead(&_D1);

	MS5611_convert(MS5611_CMD_CONV_D2);
	HAL_Delay(ct);
	MS5611_adcRead(&_D2);

	MS5611_processRawData(_D1,_D2,true);
}

void MS5611_getConvTime(uint8_t * ms_time){
	* ms_time = ct;
}

void MS5611_convert(const uint8_t addr)
{
	I2Cdev_command(MS5611_ADDRESS, addr + _samplingRate);
}

void MS5611_adcRead(uint32_t* buffer)
{
    uint8_t temp [3];
    I2Cdev_readBytes(MS5611_ADDRESS, MS5611_CMD_ADC_READ, 3, temp, 1000);
    *buffer = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];
}


void MS5611_processRawData(uint32_t D1 , uint32_t D2, bool compensation)
{
	float dT = D2 - _C[5];
	_temperature = (int32_t)(2000 + (dT * _C[6]));

	float offset =  _C[2] + (dT * _C[4]);
	float sens = _C[1] + (dT * _C[3]);

	if (compensation)
	{
		if (_temperature < 2000)
		{
			float T2 = dT * dT * 4.6566128731E-10;
			float t = (_temperature - 2000) * (_temperature - 2000);
			float offset2 = 2.5 * t;
			float sens2 = 1.25 * t;
			if (_temperature < -1500)
			{
				t = (float)((_temperature + 1500) * (_temperature + 1500));
				offset2 += 7 * t;
				sens2 += 5.5 * t;
			}
			_temperature -= T2;
			offset -= offset2;
			sens -= sens2;
		}
	}

	_pressure = (D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;
}

void MS5611_processTemperature(uint32_t D2, bool compensation)
{
    float dT = D2 - _C[5];

    _temperature = 2000 + (dT * _C[6]);

    if (compensation)
    {
    	if(_temperature < 2000){
    		float T2 = dT * dT * 4.6566128731E-10;
    		_temperature -= T2;
    	}
    }
}



void MS5611_getPressure(int32_t* buffer){
	*buffer = _pressure + _pressureOffset;
}

void MS5611_getTemperature(int32_t* buffer){
	*buffer = _temperature + _temperatureOffset;
}

void MS5611_setPressureOffset(int32_t offset)
{
	_pressureOffset = offset;
}

void MS5611_setTemperatureOffset(int32_t offset)
{
	_temperatureOffset = offset;
}

uint32_t MS5611_readRawPressure()
{
	MS5611_convert(MS5611_CMD_CONV_D1);
	HAL_Delay(ct);
	MS5611_adcRead(&_D1);
    return _D1;
}

uint32_t MS5611_readRawTemperature()
{
	MS5611_convert(MS5611_CMD_CONV_D2);
    HAL_Delay(ct);
    MS5611_adcRead(&_D2);
    return _D2;
}
