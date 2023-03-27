
#include "ks114_sonar/comms_handler.hpp"

namespace comms_handler {
    CommsHandler::CommsHandler(boost::asio::io_context& ioc)
        : ioc_(ioc)
        , serial_port_(ioc_)
    {
    }

    bool CommsHandler::setPortName(const std::string& port_name)
    {
        port_name_ = port_name;
    }

    bool CommsHandler::connect(const std::string& port_name)
    {
        // serial_port_.set_option();
        // ioc_.start
    }


} // namespace comms_handler