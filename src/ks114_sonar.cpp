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
    try
    {
        if (this->openSerialPort())
        {
            if (this->getSonarInfo())
            {
                sonar_state_ = SonarState::Started;
            }
            else
            {
                // std::cout << "Sonar at index " << sonar_index_
                //           << " is not found! \n";
                sonar_state_ = SonarState::Unstarted;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        sonar_state_ = SonarState::Error;
    }
    return (sonar_state_ == SonarState::Started);
}

bool Ks114Sonar::stop()
{
    if (serial_port_state_ == SerialPortState::PortOpened &&
        sonar_state_ == SonarState::Started)
    {
        try
        {
            Serial_.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            serial_port_state_ = SerialPortState::PortErrored;
            sonar_state_ = SonarState::Error;
        }
        if (!Serial_.isOpen())
        {
            serial_port_state_ = SerialPortState::PortUnopened;
        }
        sonar_state_ = SonarState::Unstarted;
    }
    return (sonar_state_ == SonarState::Unstarted &&
            serial_port_state_ == SerialPortState::PortUnopened);
}

SonarConfiguration Ks114Sonar::getSonarConfig() const { return sonar_config_; }

SerialPortState Ks114Sonar::getSerialPortState() const
{
    return serial_port_state_;
}
SonarState Ks114Sonar::getSonarState() const { return sonar_state_; }

bool Ks114Sonar::setIndex(const int index)
{
    if (sonar_state_ != SonarState::Started)
    {
        sonar_index_ = index;
        return true;
    }
    return false;
}

bool Ks114Sonar::setSerialPort(const std::string &new_port_name)
{
    if (serial_port_state_ != SerialPortState::PortOpened)
    {
        Serial_.setPort(new_port_name);
        return true;
    }
    return false;
}

bool Ks114Sonar::setSerialBaudRate(const int new_baud_rate)
{
    if (serial_port_state_ != SerialPortState::PortOpened)
    {
        Serial_.setBaudrate(new_baud_rate);
        return true;
    }
    return false;
}

void Ks114Sonar::sendSerialCmd(const uint8_t command[3])
{
    Serial_.write(command, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool Ks114Sonar::openSerialPort()
{
    if (serial_port_state_ != SerialPortState::PortOpened)
    {
        try
        {
            Serial_.setPort(serial_port_);
            Serial_.setBaudrate(serial_baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            Serial_.setTimeout(timeout);
            Serial_.open();
            if (Serial_.isOpen())
            {
                // std::cout << "Serial port " << serial_port_
                //           << " is opened at baud rate " << serial_baud_rate_
                //           << '\n';
                serial_port_state_ = SerialPortState::PortOpened;
            }
            else
            {
                std::cout << "Serial port " << serial_port_
                          << " failed to open at baud rate "
                          << serial_baud_rate_ << '\n';
                serial_port_state_ = SerialPortState::PortUnopened;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            Serial_.close();
            serial_port_state_ = SerialPortState::PortErrored;
        }
    }
    return (serial_port_state_ == SerialPortState::PortOpened);
}

bool Ks114Sonar::getSonarInfo()
{
    bool success{false};
    std::vector<uint8_t> byte_received{};
    uint8_t info_cmd[3]{SONAR_ADDRESSES.at(sonar_index_), 0x02, 0x99};
    Serial_.flush();
    Serial_.write(info_cmd, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (Serial_.available())
    {
        Serial_.read(byte_received, 22);
        sonar_config_.firmware_version = byte_received.at(0);
        sonar_config_.manufacturing_date = byte_received.at(1);
        sonar_config_.baud_rate = static_cast<int>(byte_received.at(4));
        sonar_config_.address = byte_received.at(5);
        sonar_config_.noise_suppression_level =
            NOISE_SUPPRESSIONS.at(byte_received.at(6));
        sonar_config_.beam_angle = BEAM_ANGLES.at(byte_received.at(7));
        sonar_config_.error_code = byte_received.at(8);
        std::cout << "Found sonar " << sonar_index_ << " connected! \n";
        success = true;
    }
    return success;
}

bool Ks114Sonar::getDistance(const DetectionMode &mode, double &distance)
{
    bool success{false};
    int data_conversion_multiplier{5800};
    if (sonar_state_ == SonarState::Started)
    {
        int sleep_time_ms{15};
        if (mode == DetectionMode::Far)
        {
            sleep_time_ms = 50;
            data_conversion_multiplier = 1000;
        }
        std::vector<uint8_t> byte_received{};
        const uint8_t command[3]{sonar_config_.address, 0x02,
                                 DETECTION_MODE.at(mode)};
        Serial_.write(command, 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
        if (Serial_.available())
        {
            Serial_.read(byte_received, 2);
            auto value_in_byte = static_cast<uint16_t>(byte_received[0] << 8 |
                                                       byte_received[1]);
            distance = static_cast<float>(value_in_byte) /
                       data_conversion_multiplier;
            success = true;
        }
    }
    return success;
}
