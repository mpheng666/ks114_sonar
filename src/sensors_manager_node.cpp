#include "sensors_manager/sensors_manager.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_manager_node");
    ros::NodeHandle nh("~");

    sensors_manager::SensorsManager sensors_manager(nh);
    sensors_manager.start();

    ros::spin();
    return 0;
}