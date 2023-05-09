#include "ks114_sonar/ks114_sonar.hpp"

int main(int argc, char** argv)
{
    using namespace ks114_sonar;
    Ks114Sonar sonar(1);
    std::cout << "Sonar config: \n";
    sonar.sonar_config.printConfig();
    std::cout << "Config command: ";
    for (const auto& val : sonar.getConfigCommand())
    {
        std::cout << std::hex << unsigned(val) << " ";
    }
    std::cout << "\n";
    std::cout << "Sense command: ";
    for (const auto& val : sonar.getSenseCommand(DetectionMode::Fast))
    {
        std::cout << std::hex << unsigned(val) << " ";
    }
    std::cout << "\n";

    return 0;
}