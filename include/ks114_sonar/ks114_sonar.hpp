#ifndef KS114_SONAR_UTILITY_HPP_
#define KS114_SONAR_UTILITY_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <serial/serial.h>

// 0x69: Program version, stored in Register 0;
// 0xa9: Mark of manufacturing date, stored in Register 1;
// 0x79: Communication baud rate of serial port, stored in Register 4;
// 0xe8: I2C or serial port address, stored in Register 5;
// 0x71: Noise reduction level, stored in Register 6;
// 0x7b: Factory default setting for configuring dead zone. Stored in Register 7;
// 0xe3: Error code, stored in Register 8;
// 0x68: Initialization completion mark, stored in Register 9; its value is 0x69 when initialization starts;

namespace ks114_sonar
{
    class Ks114Sonar
    {
    public:
        Ks114Sonar();
        ~Ks114Sonar();
        void start();
        void getFirmwareVersion();
        void getManufacturingDate();
        void getBaudRate();
        void getAddress();
        void getNoiseSuppressionLevel();
        void getBeamAngle();
        void getErrorCode();

    private:
        enum class PortState
        {
            PortUnopened,
            PortOpened,
            PortErrored
        };

        uint8_t firmware_version_;
        uint8_t manufacturing_date_;
        uint8_t baud_rate_;
        uint8_t address_;
        uint8_t noise_suppression_level_;
        uint8_t beam_angle_;
        uint8_t error_code_;
    };
}

#endif