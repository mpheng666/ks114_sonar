#include "sensors_manager/sensors_manager.hpp"

using namespace sensors_manager;

SensorsManager::SensorsManager(ros::NodeHandle &nh)
    : nh_p_(nh), sonars_pub_(nh_p_.advertise<std_msgs::Float64MultiArray>(
                         "sonars_data", 10))
{
}

SensorsManager::~SensorsManager() {}

void SensorsManager::start() {}