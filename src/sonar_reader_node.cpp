#include "ks114_sonar/sonar_reader.hpp"

int main()
{
    using namespace ks114_sonar;
    Ks114Sonar sonar(13);
    CommsHandler comms("/dev/ttyUSB0", 115200, 50, true);
    SonarReader reader(sonar, comms);
    reader.start();
    static constexpr int READ_NUM = 100;
    int read                      = 0;
    while (read < READ_NUM)
    {
        if (auto distance = reader.getDistance(DetectionMode::Far))
        {
            std::cout << "Distance: " << distance.value() << "\n";
            ++read;
        }
        else
        {
            std::cout << "Failed to get distance \n";
        }
    }

    return 0;
}