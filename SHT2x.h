#ifndef SHT2X_H_INCLUDED
#define SHT2X_H_INCLUDED
/*
 * SHT2x.h
 *
 * Author: Makoto Uju
 * Created: 2016/11/07
 */

#include <Arduino.h>
#include <math.h>

#define SHT2x_DEFAULT_TIMEOUT_MS 10

class SHT2x {
  private:
    const uint8_t ADDR = 0b1000000;

    enum class Command : uint8_t {
      TemperatureHold   = 0b11100011,
      HumidityHold      = 0b11100101,
      TemperatureUnHold = 0b11110011,
      HumidityUnHold    = 0b11110101,
      WriteUserRegistor = 0b11100110,
      ReadUserRegistor  = 0b11100111,
      SoftReset         = 0b11111110
    };

    inline float convertStToTemperature(uint16_t st) {
      return -46.85 + 175.72 * (float)st / pow(2, 16);
    }

    inline float convertSrhToHumidity(uint16_t srh) {
      return -6.0 + 125.0 * (float)srh / pow(2, 16);
    }

    uint8_t readUserRegistor(void);
    uint8_t writeUserRegistor(uint8_t value);
    uint16_t readValue(uint8_t trigger);
    // readValueAsync

  public:

    enum class Resolution : uint8_t {
      RH_12_T_14 = 0b00000000,
      RH_8_T_12  = 0b00000001,
      RH_10_T_13 = 0b10000000,
      RH_11_T_11 = 0b10000001
    };

    void begin(void);
    void reset(void);

    float readTemperature(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS);
    float readHumidity(uint16_t timeout_ms=SHT2x_DEFAULT_TIMEOUT_MS);

    bool setResolution(Resolution resolution);

    bool heaterOn(void);
    bool heaterOff(void);

    bool otpReloadOff(void);
    bool otpReloadOn(void);

    bool isEOD(void);
};

#endif
