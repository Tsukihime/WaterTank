#include "BMP280.h"
#include <string.h>
#include <util/delay.h>

extern "C" {
    #include "../i2chw/i2cmaster.h" // Include the I2C library
}

BMP280::BMP280(uint8_t address) : _address(address), _bmp280_temp(0), _bmp280_pres(0) {}

bool BMP280::init() {
    uint8_t buffer[1];
    // Initialize I2C if necessary
    i2c_init();
    // Read ID register
    readMem(BMP280_ID_REG, buffer, 1);
    if (buffer[0] != BMP280_ID_VAL) {
        return false;
    }
    getCalibration();
    setSampling(MODE_NORMAL, SAMPLING_X2, SAMPLING_X16, FILTER_OFF, STANDBY_MS_1);
    return true;
}

uint8_t BMP280::getStatus() {
    uint8_t data[1];
    readMem(BMP280_STATUS_REG, data, 1);
    return data[0];
}

void BMP280::setConfig(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en) {
    writeMem(BMP280_CONFIG_REG,
              ((t_sb & 0x7) << 5) |
              ((filter & 0x7) << 2) |
              (spi3w_en & 1));
}

void BMP280::setControl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode) {
    writeMem(BMP280_CONTROL_REG,
              ((osrs_t & 0x7) << 5) |
              ((osrs_p & 0x7) << 2) |
              (mode & 0x3));
}

void BMP280::setSampling(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling, uint8_t filter, uint8_t duration) {
    setConfig(duration, filter, 0);
    setControl(tempSampling, pressSampling, mode);
}

void BMP280::takeForcedMeasurement(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling) {
    setControl(tempSampling, pressSampling, mode);
    while (getStatus() & 0x08) { // Wait until measurement has been completed
        _delay_ms(1);
    }
    measure();
}

void BMP280::measure() {
    uint8_t data[BMP280_RAWDATA_BYTES];
    int32_t temp_raw, pres_raw, var1, var2, t_fine;

    // Read the raw ADC data from the I2C registers
    readMem(BMP280_PRES_REG, data, BMP280_RAWDATA_BYTES);
    pres_raw = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    temp_raw = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    // Compute the temperature
	var1 = ((((temp_raw >> 3) - ((int32_t)_bmp280_cal.dig_t1 << 1))) * ((int32_t)_bmp280_cal.dig_t2)) >> 11;
	var2 = (((((temp_raw >> 4) - ((int32_t)_bmp280_cal.dig_t1)) * ((temp_raw >> 4) - ((int32_t)_bmp280_cal.dig_t1))) >> 12) * ((int32_t)_bmp280_cal.dig_t3)) >> 14;
	t_fine = var1 + var2;
	_bmp280_temp = (t_fine * 5 + 128) >> 8;

    // Compute the pressure
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)_bmp280_cal.dig_p6);
	var2 = var2 + ((var1 * ((int32_t)_bmp280_cal.dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)_bmp280_cal.dig_p4) << 16);
	var1 = (((_bmp280_cal.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
	       + ((((int32_t)_bmp280_cal.dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)_bmp280_cal.dig_p1)) >> 15);

	if (var1 == 0) {
		_bmp280_pres = 0;
	} else {
		_bmp280_pres = (((uint32_t)(((int32_t)1048576)-pres_raw) - (var2 >> 12))) * 3125;
		if (_bmp280_pres < 0x80000000) {
			_bmp280_pres = (_bmp280_pres << 1) / ((uint32_t)var1);
		} else {
			_bmp280_pres = (_bmp280_pres / (uint32_t)var1) * 2;
		}
		var1 = (((int32_t)_bmp280_cal.dig_p9) * ((int32_t)(((_bmp280_pres >> 3) * (_bmp280_pres >> 3)) >> 13))) >> 12;
		var2 = (((int32_t)(_bmp280_pres >> 2)) * ((int32_t)_bmp280_cal.dig_p8)) >> 13;
		_bmp280_pres = (uint32_t)((int32_t)_bmp280_pres + ((var1 + var2 + _bmp280_cal.dig_p7) >> 4));
	}
}

void BMP280::writeMem(uint8_t reg, uint8_t value) {
    i2c_start_wait((_address << 1) | I2C_WRITE);
    i2c_write(reg);
    i2c_write(value);
    i2c_stop();
}

void BMP280::readMem(uint8_t reg, uint8_t buff[], uint8_t bytes) {
    i2c_start_wait((_address << 1) | I2C_WRITE);
    i2c_write(reg);
    i2c_rep_start((_address << 1) | I2C_READ);

    for(uint8_t i = 0; i < bytes; i++) {
        buff[i] = (i == bytes - 1) ? i2c_readNak() : i2c_readAck();
    }
    i2c_stop();
}

void BMP280::getCalibration() {
    memset(_bmp280_cal.bytes, 0, sizeof(_bmp280_cal));
    readMem(BMP280_CAL_REG_FIRST, _bmp280_cal.bytes, BMP280_CAL_DATA_SIZE);
}
