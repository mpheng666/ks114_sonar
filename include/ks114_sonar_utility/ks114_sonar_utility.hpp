#ifndef _KS114_SONAR_UTILITY_HPP_
#define _KS114_SONAR_UTILITY_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <thread>

#include <serial/serial.h>
#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar_utility
{
    enum class UtilityMode
    {
        ADDRESS_MODIFICATION,
        BAUTRATE_MODIFICATION,
        NOISE_SUPPRESSION_MODIFICATION,
        BEAM_ANGLE_MODIFICATION,
        DETECTION
    };

    class KS114SonarUtility
    {
    public:
        KS114SonarUtility();
        ~KS114SonarUtility();
        void start();

    private:
        static constexpr std::array<uint8_t, 20> SONAR_ADDRESSES_{
            0XD0, 0XD2, 0XD4, 0XD6,
            0XD8, 0XDA, 0XDC, 0XDE,
            0XE0, 0XE2, 0XE4, 0XE6,
            0XE8, 0XEA, 0XEC, 0XEE,
            0XF8, 0XFA, 0XFC, 0XFE};
        static constexpr std::array<uint8_t, 5> NOISE_SUPPRESSIONS_{
            0X71, 0X72, 0X73, 0X74, 0X75};
        static constexpr std::array<uint8_t, 8> BEAM_ANGLES_{
            0X7A, 0X7B, 0X7C, 0X7D, 
            0X7E, 0X80, 0X81, 0X82};
        std::string serial_port_{"/dev/ttyUSB0"};
        int baud_rate_{115200};
        UtilityMode UMode_;
        serial::Serial Serial_;
        uint8_t connected_sonar_{0XE8};
        uint8_t current_noise_suppression_level_ {0X71};
        uint8_t current_beam_config_{0X7B};

        bool selectMode(UtilityMode &utility_mode);
        bool openSerial(const std::string &serial_port, const int baudrate);
        bool getConnectedSensorInfo(uint8_t &output_address);
        void runStateMachine(UtilityMode &utility_mode);
        bool compareAddress(const uint8_t reference_sensor_address);
        bool setNewAddress(const uint8_t sensor_current_address);
        bool setNewBaudRate(const int baudrate = 115200);
        bool setNewNoiseSuppressionLevel(const int level = 1);
        bool setNewBeamAngleMode(const int mode = 1);
        bool detect(const int mode = 1);
    };
}

#endif