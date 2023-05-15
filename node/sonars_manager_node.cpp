#include "ks114_sonar/sonars_manager.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    using namespace ks114_sonar;
    ros::init(argc, argv, "sonars_manager_node");
    ros::NodeHandle nh("~");

    SonarsManager sonars_manager(nh);
    sonars_manager.start();

    ros::spin();
    return 0;
}