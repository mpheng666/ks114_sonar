#include "ks114_sonar/sensor_reader.hpp"

using namespace ks114_sonar;

int main()
{
    Ks114Sonar sonar(13);
    CommsHandler comms("/dev/ttyUSB0", 115200, 500);
    SensorReader reader(comms, sonar);
    reader.start();
    // reader.read(400, false);

    return 0;
}