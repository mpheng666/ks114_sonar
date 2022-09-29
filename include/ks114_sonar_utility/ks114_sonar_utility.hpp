#ifndef _KS114_SONAR_UTILITY_HPP_
#define _KS114_SONAR_UTILITY_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <thread>

#include <serial/serial.h>

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
        static constexpr std::array<uint8_t, 20> sonar_address_{
            0XD0, 0XD2, 0XD4, 0XD6,
            0XD8, 0XDA, 0XDC, 0XDE,
            0XE0, 0XE2, 0XE4, 0XE6,
            0XE8, 0XEA, 0XEC, 0XEE,
            0XF8, 0XFA, 0XFC, 0XFE};
        std::string serial_port_{"/dev/ttyUSB0"};
        int baud_rate_{115200};
        UtilityMode UMode_;
        serial::Serial Serial_;
        uint8_t connected_sonar_{0x00};

        bool selectMode(UtilityMode& utility_mode);
        bool openSerial(const std::string &serial_port, const int baudrate);
        bool getConnectedSensor(uint8_t &output_address);
        void runStateMachine(UtilityMode& utility_mode);
        bool compareAddress(const uint8_t reference_sensor_address);
        bool setNewAddress(const uint8_t sensor_current_address);
        bool setNewBaudRate(const int baudrate=115200);
        bool setNewNoiseSuppressionLevel(const int level=1);
        bool setNewBeamAngleMode(const int mode=1);
        bool detect(const int mode=1);
    };
}

#endif