#ifndef KS114_SONAR_COMMS_HANDLER_HPP_
#define KS114_SONAR_COMMS_HANDLER_HPP_

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread/thread.hpp>
#include <boost/timer/timer.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
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
                     unsigned int connection_timer_ms);
        ~CommsHandler();
        bool start();
        bool stop();
        int getCurrentConnectionState() const;
        bool write(const std::vector<uint8_t>& data,
                   unsigned int expected_return_duration_ms);
        [[nodiscard]] std::vector<uint8_t> getReadBuffer();
        void setPort(const std::string& port_name, unsigned int baud_rate);

        private:
        std::string port_name_ {};
        unsigned int baud_rate_ {115200};
        ConnectionState target_connection_state_ {ConnectionState::NOTCONNECTED};
        ConnectionState current_connection_state_ {ConnectionState::NOTCONNECTED};

        boost::asio::io_context ioc_;
        boost::asio::serial_port serial_port_;
        unsigned int connection_timer_ms_ {1000};

        boost::asio::steady_timer connection_handler_cb_timer_;

        static constexpr size_t READ_BUFFER_SIZE_ = 1024;
        static constexpr size_t MAX_READ_BUFFER_SIZE_ =
            READ_BUFFER_SIZE_ * READ_BUFFER_SIZE_;

        char read_buff_raw_[READ_BUFFER_SIZE_];
        std::vector<uint8_t> read_buff_byte_ {};

        std::mutex serial_mutex_;

        void setupSerialPort();
        void connectSerial();
        void disconnectSerial();
        void asyncRead(const boost::system::error_code& ec);
        void handleRead(const boost::system::error_code& ec,
                        std::size_t bytes_transferred);
        void handleConnection(const boost::system::error_code& ec);
    };

}  // namespace sam_mk

#endif