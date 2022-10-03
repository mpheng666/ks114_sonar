#ifndef KS114_SONAR_UTILITY_HPP_
#define KS114_SONAR_UTILITY_HPP_

#include <array>

#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar_utility {
enum class UtilityMode {
    ADDRESS_MODIFICATION,
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
    std::string serial_port_{"/dev/ttyUSB0"};
    int baud_rate_{115200};
    UtilityMode UMode_;
    const std::size_t sonars_size = ks114_sonar::SONAR_ADDRESSES.size();

    bool selectMode();
    // void runStateMachine(UtilityMode &utility_mode);
    // bool compareAddress(const uint8_t reference_sensor_address);
    // bool setNewAddress(const uint8_t sensor_current_address);
    // bool setNewBaudRate(const int baudrate = 115200);
    // bool setNewNoiseSuppressionLevel(const int level = 1);
    // bool setNewBeamAngleMode(const int mode = 1);
    // bool detect(const int mode = 1);
};
} // namespace ks114_sonar_utility

#endif