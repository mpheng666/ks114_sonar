#include "ks114_sonar/sonars_manager.hpp"

namespace ks114_sonar
{
    SonarsManager::SonarsManager(ros::NodeHandle& nh)
        : nh_p_(nh)
        , comms_handler_("/dev/ttyUSB0", 115200, 500, false)
    {
        loadParams();
    }

    void SonarsManager::start()
    {
        std::cout << "Start \n";
        // comms_handler_.start();
        std::cout << "Started \n";
        while (ros::ok())
        {
            for (auto& reader : sonars_reader_)
            {
                reader.start();
                if (auto val = reader.getDistance(DetectionMode::Fast))
                {
                    std::cout << "Distance: " << val.value() << "\n";
                }
            }
        }
    }

    void SonarsManager::loadParams()
    {
        std::cout << "Load param \n";
        for (int i = 1; i < 16; ++i)
        {
            sonars_.emplace_back(i);
        }
        for (auto& sonar : sonars_)
        {
            sonars_reader_.emplace_back(sonar, comms_handler_);
        }

        // Ks114Sonar sonar {13};
        // SonarReader reader {13, comms_handler_};
    }

}  // namespace ks114_sonar