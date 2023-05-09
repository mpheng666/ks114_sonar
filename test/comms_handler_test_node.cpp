#include "ks114_sonar/comms_handler.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace ks114_sonar;

int main(int argc, char** argv)
{
    CommsHandler comms_handler("/dev/ttyUSB0", 115200, 500, true);

    static constexpr int N = 30;

    for (int i = 0; i < N; ++i)
    {
        std::cout << "i: " << i << "\n";

        std::vector<uint8_t> req_cmd {0xe8, 0x02, 0xb0};
        if (comms_handler.write(req_cmd))
        {
            comms_handler.read();
        }

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500ms);
    }

    return 0;
}