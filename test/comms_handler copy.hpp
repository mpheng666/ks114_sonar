#ifndef KS114_SONAR_COMMS_HANDLER_HPP_
#define KS114_SONAR_COMMS_HANDLER_HPP_

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind/bind.hpp>
#include <boost/optional.hpp>
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
        void asyncRead();
        void handleConnection(const boost::system::error_code& ec);

        template<typename SyncReadStream, typename MutableBufferSequence>
        void
        readWithTimeout(SyncReadStream& s,
                        const MutableBufferSequence& buffers,
                        const boost::asio::deadline_timer::duration_type& expiry_time)
        {
            boost::optional<boost::system::error_code> timer_result;
            boost::asio::deadline_timer timer(s.get_executor());
            timer.expires_from_now(expiry_time);
            timer.async_wait([&timer_result](const boost::system::error_code& error) {
                timer_result.reset(error);
            });

            boost::optional<boost::system::error_code> read_result;
            boost::asio::async_read(s,
                                    buffers,
                                    [&read_result](const boost::system::error_code& error,
                                                   size_t) { read_result.reset(error); });

            ioc_.reset();
            while (ioc_.run_one())
            {
                if (read_result)
                    timer.cancel();
                else if (timer_result)
                    s.cancel();
            }

            if (*read_result)
                throw boost::system::system_error(*read_result);
        }
    };

}  // namespace ks114_sonar

#endif