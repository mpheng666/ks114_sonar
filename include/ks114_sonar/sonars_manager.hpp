#ifndef KS114_SONAR_SONAR_MANAGER_HPP_
#define KS114_SONAR_SONAR_MANAGER_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"
#include "ks114_sonar/sonar_reader.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace ks114_sonar
{
    class SonarsManager
    {
        public:
        SonarsManager(ros::NodeHandle& nh);
        void start();

        private:
        ros::NodeHandle nh_p_;
        ros::Publisher raw_values_pub_;
        CommsHandler comms_handler_;
        std::vector<Ks114Sonar> sonars_;
        std::vector<SonarReader> sonars_reader_;

        int sensor_num_ {8};

        void loadParams();
        void pubSonar(const std::vector<double>& readings);
    };
}  // namespace ks114_sonar

#endif