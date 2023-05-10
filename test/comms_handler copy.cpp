#include "ks114_sonar/comms_handler.hpp"

namespace ks114_sonar
{
    CommsHandler::CommsHandler()
        : serial_port_(ioc_)
        , connection_handler_cb_timer_(
              ioc_,
              std::chrono::duration(std::chrono::milliseconds(connection_timer_ms_)))
    {
        setupSerialPort();
    }

    CommsHandler::CommsHandler(const std::string& port_name,
                               const unsigned int baud_rate,
                               unsigned int connection_timer_ms = 1000)
        : port_name_(port_name)
        , baud_rate_(baud_rate)
        , serial_port_(ioc_)
        , connection_timer_ms_(connection_timer_ms)
        , connection_handler_cb_timer_(
              ioc_,
              std::chrono::duration(std::chrono::milliseconds(connection_timer_ms_)))
    {
        setupSerialPort();
    }

    CommsHandler::~CommsHandler() { ioc_.stop(); }

    void CommsHandler::setPort(const std::string& port_name, unsigned int baud_rate)
    {
        port_name_ = port_name;
        baud_rate_ = baud_rate;
        setupSerialPort();
    }

    bool CommsHandler::start()
    {
        target_connection_state_ = ConnectionState::CONNECTED;
        connection_handler_cb_timer_.expires_after(
            boost::asio::chrono::milliseconds(static_cast<int>(connection_timer_ms_)));
        connection_handler_cb_timer_.async_wait(boost::bind(
            &CommsHandler::handleConnection, this, boost::asio::placeholders::error));
        boost::thread t(boost::bind(&boost::asio::io_context::run, &ioc_));
        return !ioc_.stopped();
    }

    bool CommsHandler::stop()
    {
        target_connection_state_ = ConnectionState::NOTCONNECTED;
        disconnectSerial();
        ioc_.stop();
        ioc_.reset();
        return ioc_.stopped();
    }

    int CommsHandler::getCurrentConnectionState() const
    {
        return static_cast<int>(current_connection_state_);
    }

    bool CommsHandler::write(const std::vector<uint8_t>& data,
                             unsigned int expected_return_duration_ms = 0)
    {
        if (current_connection_state_ == ConnectionState::CONNECTED)
        {
#if DEBUG_TIMER
            const auto p1 = std::chrono::system_clock::now();
            std::cout << std::dec
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             p1.time_since_epoch())
                             .count()
                      << " Writing: ";
#endif
#if DEBUG_BUFFER
            std::cout << "Writing: | ";
            for (const auto& e : data)
            {
                std::cout << std::setw(3) << std::hex << unsigned(e) << " | ";
            }
            std::cout << "\n";
#endif
            boost::system::error_code ec;
            [[maybe_unused]] auto write_byte =
                serial_port_.write_some(boost::asio::buffer(data), ec);
            if (ec)
            {
                std::cerr << "Write error: " << ec.message() << "\n";
                current_connection_state_ = ConnectionState::ERROR;
                return false;
            }
            else
            {
#if DEBUG_BUFFER
                std::cout << "Write size: " << write_byte << "\n";
#endif
                asyncRead();
            }
        }
        return false;
    }

    void CommsHandler::asyncRead()
    {
        if (current_connection_state_ == ConnectionState::CONNECTED)
        {
#if DEBUG_TIMER
            const auto p1 = std::chrono::system_clock::now();
            std::cout << std::dec
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             p1.time_since_epoch())
                             .count()
                      << " Async reading \n";
#endif
            boost::optional<boost::system::error_code> timer_result;
            boost::asio::deadline_timer read_timeout_timer(ioc_);
            read_timeout_timer.expires_from_now(boost::posix_time::millisec(200));
            std::cout << "start timer \n";
            read_timeout_timer.async_wait(
                [&timer_result](const boost::system::error_code& error) {
                    timer_result.reset(error);
                    // if (error)
                    // {
                    //     std::cout << "timeout timer error: " << error.message() <<
                    //     "\n";
                    // }
                    // else
                    // {
                    //     std::cout << "read timed out, cancel serial port \n";
                    //     serial_port_.cancel();
                    // }
                });

            std::cout << "async read some \n";
            boost::optional<boost::system::error_code> read_result;
            serial_port_.async_read_some(
                boost::asio::buffer(read_buff_raw_, READ_BUFFER_SIZE_),
                [&read_result](const boost::system::error_code& error,
                               std::size_t bytes_transferred) {
                    read_result.reset(error);
                    // if (error)
                    // {
                    //     std::cout << "Error handling read " << error.message() << "\n";
                    // }
                    // else
                    // {
                    //     std::cout << "handling read with size: " << bytes_transferred
                    //               << "\n";
                    //     for (unsigned int i = 0; i < bytes_transferred; ++i)
                    //     {
                    //         read_buff_byte_.push_back(read_buff_raw_[i]);
                    //     }
                    // }
                });

            ioc_.reset();
            while (ioc_.run_one())
            {
                if (read_result)
                    read_timeout_timer.cancel();
                else if (timer_result)
                    serial_port_.cancel();
            }

            if (*read_result)
                throw boost::system::system_error(*read_result);
        }
    }

    std::vector<uint8_t> CommsHandler::getReadBuffer()
    {
        // std::vector<uint8_t> retval;
        std::lock_guard<std::mutex> guard(serial_mutex_);
        const std::vector<uint8_t> retval {read_buff_byte_.begin(),
                                           read_buff_byte_.end()};
        read_buff_byte_.clear();
        return retval;
    }

    void CommsHandler::setupSerialPort()
    {
        try
        {
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
        }
        catch (const std::exception& e)
        {
            std::cerr << "Serial setup error " << e.what() << '\n';
        }
    }

    void CommsHandler::connectSerial()
    {
        if (!serial_port_.is_open())
        {
            try
            {
                serial_port_.open(port_name_);
            }
            catch (const std::exception& e)
            {
                std::cerr << "Connection error with port " << port_name_ << " "
                          << e.what() << '\n';
                current_connection_state_ = ConnectionState::ERROR;
            }
        }
        else
        {
            serial_port_.close();
        }
        if (serial_port_.is_open())
        {
            current_connection_state_ = ConnectionState::CONNECTED;
        }
    }

    void CommsHandler::disconnectSerial()
    {
        if (serial_port_.is_open())
        {
            try
            {
                serial_port_.close();
            }
            catch (const std::exception& e)
            {
                std::cerr << "Disconnection error:" << e.what() << '\n';
                current_connection_state_ = ConnectionState::ERROR;
            }
        }
        if (!serial_port_.is_open())
        {
            current_connection_state_ = ConnectionState::NOTCONNECTED;
        }
    }

    void CommsHandler::handleConnection([
        [maybe_unused]] const boost::system::error_code& ec)
    {
        // std::cout << "handle connection \n";
        // std::cout << "ioc running: " << !ioc_.stopped() << "\n";
        if (target_connection_state_ != current_connection_state_)
        {
            switch (target_connection_state_)
            {
                case ConnectionState::NOTCONNECTED:
                    if (current_connection_state_ != ConnectionState::NOTCONNECTED)
                    {
                        disconnectSerial();
                    }
                    break;
                case ConnectionState::CONNECTED:
                    if (current_connection_state_ != ConnectionState::CONNECTED)
                    {
                        connectSerial();
                    }
                    break;
                case ConnectionState::ERROR:
                    if (current_connection_state_ != ConnectionState::ERROR)
                    {
                    }
                    break;
            }
        }
        std::cout << "Current connection: " << static_cast<int>(current_connection_state_)
                  << "\n";
        connection_handler_cb_timer_.expires_at(
            connection_handler_cb_timer_.expiry() +
            std::chrono::duration(std::chrono::milliseconds(connection_timer_ms_)));
        connection_handler_cb_timer_.async_wait(
            boost::bind(&CommsHandler::handleConnection, this, ec));
    }

}  // namespace ks114_sonar