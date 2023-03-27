#ifndef KS114_SONAR_COMMS_HANDLER_HPP_
#define KS114_SONAR_COMMS_HANDLER_HPP_

#include <boost/asio.hpp>

namespace comms_handler
{
    enum class CommState : int{
        OPENED,
        CLOSED,
        ERRORED
    };

    class CommsHandler
    {
        public:
            CommsHandler(boost::asio::io_context& ioc);
            bool connect(const std::string& port_name);
            bool setPortName(const std::string& port_name);

        private:
            boost::asio::io_context& ioc_;
            boost::asio::serial_port serial_port_;
            std::string port_name_;
            uint32_t baudrate_;
    };

}

#endif