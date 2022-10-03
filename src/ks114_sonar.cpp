#include "ks114_sonar/ks114_sonar.hpp"

using namespace ks114_sonar;

Ks114Sonar::Ks114Sonar(const int index,
                       const std::string serial_port,
                       const int serial_baud_rate)
    : sonar_index_(index), serial_port_(serial_port),
      serial_baud_rate_(serial_baud_rate)
{
}

Ks114Sonar::~Ks114Sonar() {}

bool Ks114Sonar::start()
{
    try {
        if (this->openSerialPort()) {
            if (this->getSonarInfo()) {
                sonar_state_ = SonarState::Started;
            }
            else {
                std::cerr << "Please connect a sonar and restart!";
                sonar_state_ = SonarState::Unstarted;
            }
        }
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        sonar_state_ = SonarState::Error;
    }
    return (sonar_state_ == SonarState::Started);
}

inline SonarConfiguration Ks114Sonar::getSonarConfig() const
{
    return SonarConfig_;
}

inline SerialPortState Ks114Sonar::getSerialPortState() const
{
    return serial_port_state_;
}
inline SonarState Ks114Sonar::getSonarState() const { return sonar_state_; }

bool Ks114Sonar::openSerialPort()
{
    try {
        Serial_.setPort(serial_port_);
        Serial_.setBaudrate(serial_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        Serial_.setTimeout(timeout);
        Serial_.open();
        if (Serial_.isOpen()) {
            std::cout << "Serial port " << serial_port_
                      << " is opened at baud rate " << serial_baud_rate_
                      << '\n';
            serial_port_state_ = SerialPortState::PortOpened;
        }
        else {
            std::cerr << "Serial port " << serial_port_
                      << " failed to open at baud rate " << serial_baud_rate_
                      << '\n';
            serial_port_state_ = SerialPortState::PortUnopened;
        }
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        Serial_.close();
        serial_port_state_ = SerialPortState::PortErrored;
    }
    return (serial_port_state_ == SerialPortState::PortOpened);
}

bool Ks114Sonar::getSonarInfo()
{
    bool success{false};
    std::vector<uint8_t> byte_received{};
    uint8_t info_cmd[3]{SONAR_ADDRESSES.at(sonar_index_), 0x02, 0x99};
    Serial_.write(info_cmd, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    Serial_.flush();
    if (Serial_.available()) {
        Serial_.read(byte_received, 22);
        SonarConfig_.firmware_version = byte_received.at(0);
        SonarConfig_.manufacturing_date = byte_received.at(1);
        SonarConfig_.baud_rate = byte_received.at(4);
        SonarConfig_.address = byte_received.at(5);
        SonarConfig_.noise_suppression_level = byte_received.at(6);
        SonarConfig_.beam_angle = byte_received.at(7);
        SonarConfig_.error_code = byte_received.at(8);
        std::cout << "Found sensor " << sonar_index_ << "connected! \n";
        success = true;
    }
    return success;
}

bool Ks114Sonar::getDistance(const DetectionMode &mode, double &distance)
{
    bool success{false};

    if (sonar_state_ == SonarState::Started) {
        int sleep_time_ms{15};
        if (mode == DetectionMode::Far) {
            sleep_time_ms = 50;
        }
        std::vector<uint8_t> byte_received{};
        const uint8_t command[3]{SonarConfig_.address, 0x02,
                                 DETECTION_MODE.at(mode)};
        Serial_.write(command, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        if (Serial_.available()) {
            Serial_.read(byte_received, 2);
            auto value_in_byte = static_cast<uint16_t>(byte_received[0] << 8 |
                                                       byte_received[1]);
            distance = static_cast<float>(value_in_byte) / 5800.0;
            success = true;
        }
    }
    return success;
}
