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
            auto result = comms_handler_.read(2);
            if (auto distance_m = sonar_.decodeDistance(result, mode))
            {
                // std::cout << "Distance: " << distance_m.value() << "\n";

                return distance_m;
            }
        }
        // std::cout << "Failed to retrieve distance \n";
        return std::nullopt;
    }

}  // namespace ks114_sonar