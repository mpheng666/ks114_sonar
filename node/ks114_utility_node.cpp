#include "ks114_sonar/ks114_sonar_utility.hpp"

int main(int argc, char** argv)
{
    using namespace ks114_sonar;
    if (argc == 2)
    {
        const std::string port(argv[1]);
        std::cout << "Opening port " << port << "\n";
        KS114SonarUtility utility(port);
        utility.start();
    }
    else
    {
        std::cerr << "Please provide serial port name \n";
        std::cerr << "For example, <utility_node> /dev/ttyUSB0 \n";
    }

    return 0;
}