#ifndef KS114_SONAR_UTILITY_HPP_
#define KS114_SONAR_UTILITY_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"
#include "ks114_sonar/sonar_reader.hpp"

namespace ks114_sonar
{
    enum class UtilityMode
    {
        ADDRESS_MODIFICATION           = 1,
        BAUTRATE_MODIFICATION          = 2,
        NOISE_SUPPRESSION_MODIFICATION = 3,
        BEAM_ANGLE_MODIFICATION        = 4,
        DETECTION                      = 5,
        AUTO_SEARCH_SONAR              = 6
    };

    class KS114SonarUtility
    {
        public:
        KS114SonarUtility(const std::string& port_name);
        ~KS114SonarUtility();
        void start();

        private:
        CommsHandler comms_handler_;
        std::string serial_port_ {"/dev/ttyUSB0"};
        static constexpr int baud_rate_ {115200};
        UtilityMode UMode_;
        // const std::size_t sonars_size = ks114_sonar::SONAR_ADDRESSES.size();
        int connected_sonar_index_ {1};

        int selectMode();
        void runStateMachine();
        bool setNewAddress();
        bool setNewBaudRate();
        bool setNewNoiseSuppressionLevel();
        bool setNewBeamAngleMode();
    };
}  // namespace ks114_sonar

#endif