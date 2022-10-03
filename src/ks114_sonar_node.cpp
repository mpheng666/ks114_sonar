#include "ks114_sonar/ks114_sonar.hpp"
#include <list>
#include <array>

int main(int argc, char **argv) {

    ks114_sonar::Ks114Sonar sonar_front(1);
    sonar_front.start();
    std::cout << sonar_front.getSonarConfig().beam_angle << "\n";
    while (sonar_front.getSonarState() == ks114_sonar::SonarState::Started) {

        double distance{};

        if (sonar_front.getDistance(ks114_sonar::DetectionMode::Fast,
                                    distance)) {
            std::cout << "Distance measured: " << distance << "\n";
        } else {
            std::cout << "Failed to get sonar measurement"
                      << "\n";
        }
    }

    return 0;
}