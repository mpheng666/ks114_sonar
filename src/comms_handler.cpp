#include "ks114_sonar/comms_handler.hpp"

namespace ks114_sonar
{
    CommsHandler::CommsHandler() { setupSerialPort(); }

    CommsHandler::CommsHandler(const std::string& port_name,
                               const unsigned int baud_rate,
                               unsigned int connection_timer_ms = 1000,
                               bool use_autoconnect             = true)
        : port_name_(port_name)
        , baud_rate_(baud_rate)
        , connection_timer_ms_(connection_timer_ms)
        , use_autoconnect_(use_autoconnect)
    {
        setupSerialPort();
    }

    CommsHandler::~CommsHandler() { }

    void CommsHandler::setPort(const std::string& port_name, unsigned int baud_rate)
    {
        port_name_ = port_name;
        baud_rate_ = baud_rate;
        setupSerialPort();
    }

    void CommsHandler::setupSerialPort()
    {
        try
        {
            serial_port_.setPort(port_name_);
            serial_port_.setBaudrate(baud_rate_);
            serial_port_.setTimeout(10, 1000, 1, 100, 1);
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    bool CommsHandler::start() { return connectSerial(); }

    bool CommsHandler::stop() { return disconnectSerial(); }

    bool CommsHandler::isPortOpen() const
    {
        return connection_status_ == ConnectionState::CONNECTED;
    }

    bool CommsHandler::write(const std::vector<uint8_t>& data)
    {
        if (connection_status_ != ConnectionState::CONNECTED)
        {
            if (use_autoconnect_)
            {
                connectSerial();
            }
            return false;
        }

        try
        {
            serial_port_.flush();
            serial_port_.flushOutput();
            [[maybe_unused]] auto write_byte = serial_port_.write(data);
            // std::cout << "Written size: " << write_byte << "\n";
        }
        catch (const std::exception& e)
        {
            std::cerr << "SERIAL WRITE ERROR: " << e.what() << '\n';
            connection_status_ = ConnectionState::ERROR;
            return false;
        }
        return true;
    }

    std::vector<uint8_t> CommsHandler::read()
    {
        std::vector<uint8_t> retval {};
        if (connection_status_ != ConnectionState::CONNECTED)
        {
            if (use_autoconnect_)
            {
                connectSerial();
            }
            return retval;
        }
        serial_port_.flush();
        serial_port_.flushInput();

        if (serial_port_.waitReadable())
        {
            try
            {
                serial_port_.read(retval, 100);
            }
            catch (const std::exception& e)
            {
                std::cerr << "SERIAL READ ERROR: " << e.what() << '\n';
                connection_status_ = ConnectionState::ERROR;
                if (use_autoconnect_)
                {
                    connectSerial();
                }
            }
        }
        for (const auto& val : retval)
        {
            std::cout << "Read: " << std::hex << unsigned(val) << "\n";
        }
        return retval;
    }

    bool CommsHandler::connectSerial()
    {
        int time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - last_reconnection_ts_)
                            .count();
        if (connection_status_ != ConnectionState::CONNECTED &&
            time_diff > connection_timer_ms_)
        {
            std::lock_guard<std::mutex> lg(serial_mutex_);
            try
            {
                std::cout << "ATTEMPT TO CONNECT SERIAL PORT: " << port_name_ << "\n";
                last_reconnection_ts_ = std::chrono::system_clock::now();
                setupSerialPort();
                serial_port_.open();
            }
            catch (const std::exception& e)
            {
                std::cerr << "SERIAL OPEN ERROR: " << e.what() << '\n';
                serial_port_.close();
                connection_status_ = ConnectionState::ERROR;
            }
            if (serial_port_.isOpen())
            {
                std::cout << "SERIAL PORT: " << port_name_ << " CONNECTED \n";
                connection_status_ = ConnectionState::CONNECTED;
            }
        }
        return connection_status_ == ConnectionState::CONNECTED;
    }

    bool CommsHandler::disconnectSerial()
    {
        serial_port_.close();
        connection_status_ = ConnectionState::NOTCONNECTED;
        return !serial_port_.isOpen();
    }

}  // namespace ks114_sonar