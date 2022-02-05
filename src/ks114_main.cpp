#include <ros/ros.h>
#include "ks114.hpp"

const std::string ROS_NODE = "ks114_sonar_node";
const std::string PUB_TOPIC_RAW = "ks114_sonar_data_raw";
const std::string PUB_TOPIC_AUXI = "ks114_sonar_data_auxi";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_NODE);

    ks114_ns::SonarKs114 ks114_node(PUB_TOPIC_RAW, PUB_TOPIC_AUXI);
    ks114_node.startProcess();

    ros::spin();
    return 0;
}