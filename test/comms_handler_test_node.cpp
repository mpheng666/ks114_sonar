#include "ks114_sonar/comms_handler.hpp"

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <chrono>
#include <thread>

using namespace ks114_sonar;

void handler(const boost::system::error_code& error, int signal_number)
{
    std::cout << "handling signal " << signal_number << std::endl;
    exit(1);
}

int main(int argc, char** argv)
{
    boost::asio::io_context ioc;
    CommsHandler comms_handler("/dev/ttyUSB0", 115200, 1000);

    boost::asio::signal_set signals(ioc, SIGINT);
    signals.async_wait(handler);
    static constexpr int N = 10;

    if (comms_handler.start())
    {
        for (int i = 0; i < N; ++i)
        {
            std::cout << "i: " << i << "\n";
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
            std::vector<uint8_t> req_cmd {0xe8, 0x02, 0xb0};
            comms_handler.write(req_cmd, 100);
        }
    }
    else
    {
        std::cout << "Failed to connect device \n";
    }

    return 0;
}