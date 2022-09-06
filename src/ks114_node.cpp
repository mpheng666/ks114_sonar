#include <ros/ros.h>
#include "ks114/ks114.hpp"

static constexpr char ROS_NODE[] {"ks114_sonar_node"};
// static constexpr char PUB_TOPIC_RAW[] {"ks114_sonar_data_raw"};
// static constexpr char PUB_TOPIC_AUXI[] {"ks114_sonar_data_auxi"};

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_NODE);
    ros::NodeHandle nh("~");

    ks114_ns::SonarKs114 ks114_node(nh);
    ks114_node.startProcess();

    ros::spin();
    return 0;
}