#ifndef KS114_SONAR_COMMS_HANDLER_HPP_
#define KS114_SONAR_COMMS_HANDLER_HPP_

#include <serial/serial.h>
// #include <boost/asio.hpp>

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#define DEBUG_TIMER 0
#define DEBUG_BUFFER 1

namespace ks114_sonar
{
    enum class ConnectionState : int
    {
        NOTCONNECTED = 0,
        CONNECTED    = 1,
        ERROR        = 2
    };

    class CommsHandler
    {
        public:
        CommsHandler();

        CommsHandler(const std::string& port_name,
                     const unsigned int baud_rate,
                     unsigned int serial_timeout_ms,
                     bool use_autoconnect);
        ~CommsHandler();

        bool start();
        bool stop();
        bool isPortOpen() const;
        bool write(const std::vector<uint8_t>& data);
        std::vector<uint8_t> read();
        void setPort(const std::string& port_name, unsigned int baud_rate);

        private:
        std::string port_name_ {};
        unsigned int baud_rate_ {115200};
        ConnectionState connection_status_ {ConnectionState::NOTCONNECTED};
        serial::Serial serial_port_;
        unsigned int serial_timeout_ms_ {100};
        static constexpr int CONNECTION_TIMER_MS_ {1000};
        std::chrono::system_clock::time_point last_reconnection_ts_ {
            std::chrono::system_clock::now()};
        bool use_autoconnect_ {true};
        std::mutex serial_mutex_;

        void setupSerialPort();
        bool connectSerial();
        bool disconnectSerial();
    };

}  // namespace ks114_sonar

#endif