#ifndef KS114_SONAR_COMMS_HANDLER_HPP_
#define KS114_SONAR_COMMS_HANDLER_HPP_

#include <boost/asio.hpp>

#include <iostream>

namespace comms_handler {
    enum class CommState : int { OPENED, CLOSED, ERRORED };

    class CommsHandler {
    public:
        CommsHandler(boost::asio::io_context& ioc,
                     const std::string& port_name = "/dev/ttyUSB0",
                     uint32_t baudrate = 9600);
        bool connect(const std::string& port_name);
        bool connect();
        bool disconnect();
        std::string getPortName() const;
        

    private:
        boost::asio::io_context& ioc_;
        boost::asio::serial_port serial_port_;
        std::string port_name_{};
        uint32_t baudrate_{115200};

        void initSetting();
    };

} // namespace comms_handler

#endif