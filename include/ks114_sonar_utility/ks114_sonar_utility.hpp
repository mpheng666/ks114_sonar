#ifndef KS114_SONAR_UTILITY_HPP_
#define KS114_SONAR_UTILITY_HPP_

#include <array>

#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar_utility {
enum class UtilityMode {
    ADDRESS_MODIFICATION = 1,
    BAUTRATE_MODIFICATION,
    NOISE_SUPPRESSION_MODIFICATION,
    BEAM_ANGLE_MODIFICATION,
    DETECTION
};

class KS114SonarUtility {
public:
    KS114SonarUtility();
    ~KS114SonarUtility();
    void start();

private:
    std::array<ks114_sonar::Ks114Sonar, 20> sonars_;

    std::string serial_port_{"/dev/ttyUSB0"};
    int baud_rate_{115200};
    UtilityMode UMode_;
    const std::size_t sonars_size = ks114_sonar::SONAR_ADDRESSES.size();
    int connected_sonar_index_{};

    bool selectMode();
    void runStateMachine();
    bool setNewAddress();
    bool setNewBaudRate();
    bool setNewNoiseSuppressionLevel();
    bool setNewBeamAngleMode();
};
} // namespace ks114_sonar_utility

#endif