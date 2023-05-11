#include "ks114_sonar/sonars_manager.hpp"

namespace ks114_sonar
{
    SonarsManager::SonarsManager(ros::NodeHandle& nh)
        : nh_p_(nh)
        , raw_values_pub_(
              nh_p_.advertise<std_msgs::Float64MultiArray>("raw_readings", 10))
        , comms_handler_("/dev/ttyUSB0", 115200, 50, true)
    {
        loadParams();
    }

    void SonarsManager::start()
    {
        while (ros::ok())
        {
            std::vector<double> raw_readings(sensor_num_);
            for (int i = 0; i < sensor_num_; ++i)
            {
                sonars_reader_.at(i).start();
                if (auto val = sonars_reader_.at(i).getDistance(DetectionMode::Fast))
                {
                    raw_readings.at(i) = val.value();
                }
                else
                {
                    raw_readings.at(i) = 404.0;
                }
            }
            pubSonar(raw_readings);
            ros::spinOnce();
        }
    }

    void SonarsManager::loadParams()
    {
        for (int i = 0; i < sensor_num_; ++i)
        {
            sonars_.emplace_back(i + 1);
        }
        for (auto& sonar : sonars_)
        {
            sonars_reader_.emplace_back(sonar, comms_handler_);
        }
    }

    void SonarsManager::pubSonar(const std::vector<double>& readings)
    {
        std_msgs::Float64MultiArray msg;
        msg.data = readings;
        raw_values_pub_.publish(msg);
    }

}  // namespace ks114_sonar