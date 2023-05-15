#ifndef KS114_SONAR_SONAR_MANAGER_HPP_
#define KS114_SONAR_SONAR_MANAGER_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"
#include "ks114_sonar/signal_filter.hpp"
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
        ros::Publisher filtered_values_pub_;

        CommsHandler comms_handler_;
        std::vector<Ks114Sonar> sonars_;
        std::vector<SonarReader> sonars_reader_;
        std::vector<int> sonars_remappper_ {};
        std::vector<double> previous_filtered_values_;

        int detection_mode_ {0};
        static constexpr double ERROR_READING_ {6.0};
        static constexpr unsigned int BAUDRATE_ {115200};
        int comms_timeout_ms_ {100};
        std::string serial_port_name_ {"/dev/ttyUSB0"};
        bool use_low_pass_filter_ {true};
        double low_pass_gain_ {0.8};

        void loadParams();
        void pubReadings(const std::vector<double>& readings);
    };
}  // namespace ks114_sonar

#endif