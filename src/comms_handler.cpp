
#include "ks114_sonar/comms_handler.hpp"

namespace comms_handler {
    CommsHandler::CommsHandler(boost::asio::io_context& ioc,
                               const std::string& port_name = "/dev/ttyUSB0",
                               uint32_t baudrate = 9600)
        : ioc_(ioc)
        , serial_port_(ioc_)
        , port_name_(port_name)
        , baudrate_(baudrate)
    {
        initSetting();
    }

    std::string CommsHandler::getPortName() const { return port_name_; }

    bool CommsHandler::connect(const std::string& port_name)
    {
        boost::system::error_code ec;
        serial_port_.open(port_name, ec);
        if (!ec) {
            std::thread t([&] { ioc_.run(); });
            return serial_port_.is_open();
        }
        else {
            std::cerr << "Error connecting: " << ec.message() << "\n";
            return false;
        }
    }

    bool CommsHandler::connect()
    {
        boost::system::error_code ec;
        serial_port_.open(port_name_, ec);
        if (!ec) {
            std::thread t([&] { ioc_.run(); });
            return serial_port_.is_open();
        }
        else {
            std::cerr << "Error connecting: " << ec.message() << "\n";
            return false;
        }
    }

    bool CommsHandler::disconnect()
    {   
        serial_port_.close();
        ioc_.stop();
        return !serial_port_.is_open();
    }

    void CommsHandler::initSetting()
    {
        using namespace boost::asio;
        serial_port_.set_option(serial_port_base::baud_rate(baudrate_));
        serial_port_.set_option(serial_port_base::character_size(8));
        serial_port_.set_option(
        boost::asio::serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_port_.set_option(
        boost::asio::serial_port_base::parity(serial_port_base::parity::none));
        serial_port_.set_option(
        serial_port_base::flow_control(serial_port_base::flow_control::none));
    }

} // namespace comms_handler