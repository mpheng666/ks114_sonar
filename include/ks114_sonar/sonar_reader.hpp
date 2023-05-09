#ifndef KS114_SONAR_SONAR_READER_HPP_
#define KS114_SONAR_SONAR_READER_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar
{
    class SonarReader
    {
        public:
        explicit SonarReader(Ks114Sonar& sonar, CommsHandler& comms);
        ~SonarReader();
        std::optional<double> getDistance(DetectionMode mode);
        void start();

        private:
        CommsHandler& comms_handler_;
        Ks114Sonar sonar_;
        double read_frequency_ms_ {500.0};
    };

}  // namespace ks114_sonar

#endif
