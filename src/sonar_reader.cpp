#include "ks114_sonar/sonar_reader.hpp"

namespace ks114_sonar
{
    SonarReader::SonarReader(Ks114Sonar& sonar, CommsHandler& comms)
        : sonar_(sonar)
        , comms_handler_(comms)
    {
    }

    SonarReader::~SonarReader() { }

    void SonarReader::start()
    {
        while (!comms_handler_.isPortOpen())
        {
            comms_handler_.start();
        }
    }

    std::optional<double> SonarReader::getDistance(DetectionMode mode)
    {
        if (comms_handler_.write(sonar_.getSenseCommand(mode)))
        {
            auto result = comms_handler_.read();
            if (auto distance_m = sonar_.decodeDistance(result, mode))
            {
                return distance_m;
            }
        }
        return std::nullopt;
    }

}  // namespace ks114_sonar