#include "ks114_sonar/ks114_sonar.hpp"

int main(int argc, char **argv)
{
    ks114_sonar::Ks114Sonar sonar_front(1);
    if (sonar_front.start()) {
        double distance{};
        if (sonar_front.getDistance(ks114_sonar::DetectionMode::Fast,
                                    distance)) {
            std::cout << "Distance measured: " << distance << "\n";
        }
        else {
            std::cout << "Failed to get sonar measurement"
                      << "\n";
        }
    }

    return 0;
}