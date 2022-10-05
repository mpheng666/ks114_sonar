#ifndef KS114_SONAR_HPP_
#define KS114_SONAR_HPP_

#include <serial/serial.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace ks114_sonar {

struct SonarConfiguration {
    uint8_t firmware_version{};
    uint8_t manufacturing_date{};
    int baud_rate{};
    uint8_t address{};
    std::string noise_suppression_level{};
    std::string beam_angle{};
    uint8_t error_code{};

    void printConfig()
    {
        // std::cout << "Baud_rate: " << baud_rate << "\n";
        std::cout << "Address                 : " << std::hex
                  << static_cast<int>(address) << "\n";
        std::cout << "Noise_suppression_level : " << noise_suppression_level
                  << "\n";
        std::cout << "Beam_angle              : " << beam_angle << "\n";
    }
};

const std::map<int, uint8_t> SONAR_ADDRESSES{
        {1, 0XD0},  {2, 0XD2},  {3, 0XD4},  {4, 0XD6},  {5, 0XD8},
        {6, 0XDA},  {7, 0XDC},  {8, 0XDE},  {9, 0XE0},  {10, 0XE2},
        {11, 0XE4}, {12, 0XE6}, {13, 0XE8}, {14, 0XEA}, {15, 0XEC},
        {16, 0XEE}, {17, 0XF8}, {18, 0XFA}, {19, 0XFC}, {20, 0XFE}};

const std::map<uint8_t, std::string_view> NOISE_SUPPRESSIONS{
        {0X70, "LEVEL_0_BATTERY_POWERED"},
        {0X71, "LEVEL_1_BATTERY_POWERED_DEFAULT"},
        {0X72, "LEVEL_2_USB_POWERED"},
        {0X73, "LEVEL_4_USB_POWERED_LONG_DISTANCE"},
        {0X74, "LEVEL_5_SWITCHING_POWERED"},
        {0X75, "LEVEL_6_SWITCHING_POWERED"}};

const std::map<uint8_t, std::string_view> BEAM_ANGLES{
        {0X7A, "MODE_0"},  {0X7B, "MODE_1_DEFAULT"}, {0X7C, "MODE_2"},
        {0X7D, "MODE_3"},  {0X7E, "MODE_4"},         {0X80, "H100_V50"},
        {0X81, "H90_V40"}, {0X82, "H80_V35"}};

enum class SerialPortState { PortUnopened, PortOpened, PortErrored };
enum class SonarState { Unstarted, Started, Error };
enum class DetectionMode { Far, Fast };

const std::map<DetectionMode, uint8_t> DETECTION_MODE{
        {DetectionMode::Far, 0xB0}, {DetectionMode::Fast, 0x0F}};

static constexpr double DATA_GIVEN_IF_ERROR{404};
class Ks114Sonar {
public:
    Ks114Sonar(const int index = 1,
               const std::string serial_port = "/dev/ttyUSB0",
               const int serial_baud_rate = 115200);
    ~Ks114Sonar();
    bool start();
    bool stop();
    SonarConfiguration getSonarConfig() const;
    SerialPortState getSerialPortState() const;
    SonarState getSonarState() const;
    bool getDistance(const DetectionMode &mode, double &distance);
    bool setIndex(const int index);
    bool setSerialPort(const std::string &new_port_name);
    bool setSerialBaudRate(const int new_baud_rate);
    void sendSerialCmd(const uint8_t command[3]);

private:
    int sonar_index_{0};
    SonarConfiguration sonar_config_;
    std::string serial_port_{"/dev/ttyUSB0"};
    int serial_baud_rate_{115200};
    serial::Serial Serial_;
    SerialPortState serial_port_state_{SerialPortState::PortUnopened};
    SonarState sonar_state_{SonarState::Unstarted};

    bool openSerialPort();
    bool getSonarInfo();
};

} // namespace ks114_sonar

#endif