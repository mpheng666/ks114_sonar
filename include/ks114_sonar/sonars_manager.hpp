#ifndef KS114_SONAR_SONAR_MANAGER_HPP_
#define KS114_SONAR_SONAR_MANAGER_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"
#include "ks114_sonar/sonar_reader.hpp"
#include "ks114_sonar/signal_filter.hpp"

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
        ros::Publisher filtered_values_pub_;
        CommsHandler comms_handler_;
        std::vector<Ks114Sonar> sonars_;
        std::vector<SonarReader> sonars_reader_;
        std::vector<double> previous_filtered_values_;

        int sensor_num_ {8};
        double low_pass_gain_{0.8};
        static constexpr double ERROR_READING_ {6.0};

        void loadParams();
        void pubReadings(const std::vector<double>& readings);
    };
}  // namespace ks114_sonar

#endif