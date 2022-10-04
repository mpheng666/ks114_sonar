#ifndef SENSORS_MANAGER_HPP_
#define SENSORS_MANAGER_HPP_

#include "ks114_sonar/ks114_sonar.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <ros/ros.h>

namespace sensors_manager {
class SensorsManager {
public:
    SensorsManager(ros::NodeHandle &nh);
    ~SensorsManager();
    void start();

private:
    ros::NodeHandle nh_p_;
    ros::Publisher sonars_pub_;
};
} // namespace sensors_manager

#endif