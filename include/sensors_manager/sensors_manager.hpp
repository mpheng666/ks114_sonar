#ifndef SENSORS_MANAGER_HPP_
#define SENSORS_MANAGER_HPP_

#include "ks114_sonar/ks114_sonar.hpp"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <boost/range/adaptors.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace sensors_manager {
class SensorsManager {
public:
    explicit SensorsManager(ros::NodeHandle &nh);
    ~SensorsManager();
    void start();

private:
    ros::NodeHandle nh_p_;
    ros::Publisher sonars_pub_;
    ros::Timer pub_timer_;
    static constexpr double LOOP_RATE_{20.0};
    int num_of_sensor_{8};
    std::string serial_port_{"/dev/ttyUSB0"};
    int serial_baud_rate_{115200};

    std::array<ks114_sonar::Ks114Sonar, 20> sonars_;
    int detection_mode_{0};
    ks114_sonar::DetectionMode ks114_detection_mode_{
            ks114_sonar::DetectionMode::Fast};

    void loadParams();
    void startSensors();
    void timerCallBack(const ros::TimerEvent &);
};
} // namespace sensors_manager

#endif