#include "ks114_sonar/comms_handler.hpp"

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>

#include <chrono>
#include <thread>

using namespace comms_handler;

void handler(const boost::system::error_code& error, int signal_number)
{
    std::cout << "handling signal " << signal_number << std::endl;
    exit(1);
}

int main(int argc, char** argv)
{
    boost::asio::io_context ioc;
    CommsHandler comms_handler(ioc, "/dev/ttyACM0", 115200);

    boost::asio::signal_set signals(ioc, SIGINT);
    signals.async_wait(handler);

    while (true) {
        if (!comms_handler.isConnected()) {
            comms_handler.connect();
        }
        else {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
            comms_handler.read();
        }
    }

    comms_handler.disconnect();

    return 0;
}