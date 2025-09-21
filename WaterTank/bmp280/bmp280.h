#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include <stdbool.h>

// Oversampling rates
const uint8_t SAMPLING_NONE = 0x00;
const uint8_t SAMPLING_X1 = 0x01;
const uint8_t SAMPLING_X2 = 0x02;
const uint8_t SAMPLING_X4 = 0x03;
const uint8_t SAMPLING_X8 = 0x04;
const uint8_t SAMPLING_X16 = 0x05;

// Operating modes
const uint8_t MODE_SLEEP = 0x00;
const uint8_t MODE_FORCED = 0x01;
const uint8_t MODE_NORMAL = 0x03;

// Filter levels
const uint8_t FILTER_OFF = 0x00;
const uint8_t FILTER_X2 = 0x01;
const uint8_t FILTER_X4 = 0x02;
const uint8_t FILTER_X8 = 0x03;
const uint8_t FILTER_X16 = 0x04;

// Standby durations
const uint8_t STANDBY_MS_1 = 0x00;
const uint8_t STANDBY_MS_63 = 0x01;
const uint8_t STANDBY_MS_125 = 0x02;
const uint8_t STANDBY_MS_250 = 0x03;
const uint8_t STANDBY_MS_500 = 0x04;
const uint8_t STANDBY_MS_1000 = 0x05;
const uint8_t STANDBY_MS_2000 = 0x06;
const uint8_t STANDBY_MS_4000 = 0x07;

const uint8_t BMP280_ID_REG = 0xD0;
const uint8_t BMP280_ID_VAL = 0x58;
const uint8_t BMP280_CAL_REG_FIRST = 0x88;
const uint8_t BMP280_CAL_REG_LAST = 0xA1;
const uint8_t BMP280_CAL_DATA_SIZE = BMP280_CAL_REG_LAST - BMP280_CAL_REG_FIRST + 1;
const uint8_t BMP280_STATUS_REG = 0xF3;
const uint8_t BMP280_CONTROL_REG = 0xF4;
const uint8_t BMP280_CONFIG_REG = 0xF5;
const uint8_t BMP280_PRES_REG = 0xF7;
const uint8_t BMP280_TEMP_REG = 0xFA;
const uint8_t BMP280_RAWDATA_BYTES = 6; // 3 bytes pressure, 3 bytes temperature

union BMP280_CAL_DATA {
    uint8_t bytes[BMP280_CAL_DATA_SIZE];
    struct {
        uint16_t dig_t1;
        int16_t  dig_t2;
        int16_t  dig_t3;
        uint16_t dig_p1;
        int16_t  dig_p2;
        int16_t  dig_p3;
        int16_t  dig_p4;
        int16_t  dig_p5;
        int16_t  dig_p6;
        int16_t  dig_p7;
        int16_t  dig_p8;
        int16_t  dig_p9;
    };
};

/**
 * @class BMP280
 * @brief Class for interacting with BMP280 sensor for temperature and pressure measurement.
 */
class BMP280 {
public:
    /**
     * @brief Constructor for the BMP280 class.
     * @param address I2C address of the BMP280 sensor (default is 0x76).
     */
    BMP280(uint8_t address = 0x76);

    /**
     * @brief Initializes the BMP280 sensor.
     * @return True if initialization was successful, false otherwise.
     */
    bool init();
	
    uint8_t getStatus();
    void setConfig(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
    void setControl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
    void setSampling(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling, uint8_t filter, uint8_t duration);
    void takeForcedMeasurement(uint8_t mode, uint8_t tempSampling, uint8_t pressSampling);
    void measure();

    /**
     * @brief Gets the last measured temperature.
     * @return Temperature in 0.01 degrees Celsius. For example, a value of 5123 corresponds to 51.23 °C.
     */
    int32_t getTemperature() const { return _bmp280_temp; }

    /**
     * @brief Gets the last measured pressure.
     * @return Pressure in Pascals as an unsigned 32-bit integer. For example, a value of 96386 corresponds to 96386 Pa = 963.86 hPa.
     */
    uint32_t getPressurePa() const { return _bmp280_pres; }

    const BMP280_CAL_DATA* getCalibrationData() const { return &_bmp280_cal; }
        
protected:
    void writeMem(uint8_t reg, uint8_t value);
    void readMem(uint8_t reg, uint8_t buff[], uint8_t bytes);
    void getCalibration();
    
    uint8_t _address;
    int32_t _bmp280_temp;
    uint32_t _bmp280_pres;
    union BMP280_CAL_DATA _bmp280_cal;
};

#endif /* BMP280_H_ */
