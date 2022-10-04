#include "sensors_manager/sensors_manager.hpp"

using namespace sensors_manager;

SensorsManager::SensorsManager(ros::NodeHandle &nh)
    : nh_p_(nh),
      sonars_pub_(nh_p_.advertise<std_msgs::Float64>("sonars_data", 10)),
      pub_timer_(nh_p_.createTimer(
              ros::Duration(0.05), &SensorsManager::timerPubCallBack, this))

{
}

SensorsManager::~SensorsManager() {}

void SensorsManager::start()
{
    loadParams();
    startSensors();
    initRosPub();
    ros::Rate rate = LOOP_RATE_;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}

void SensorsManager::loadParams()
{
    if (!nh_p_.param("detection_mode", detection_mode_, detection_mode_)) {
        ROS_WARN_STREAM("detection_mode is not set! Use default "
                        << detection_mode_);
    }
    ks114_detection_mode_ =
            (detection_mode_ == 0)
                    ? ks114_sonar::DetectionMode::Far
                    : (detection_mode_ == 1) ? ks114_sonar::DetectionMode::Fast
                                             : ks114_sonar::DetectionMode::Fast;
    if (!nh_p_.param("num_of_sensor", num_of_sensor_, num_of_sensor_)) {
        ROS_WARN_STREAM("num_of_sensor is not set! Use default "
                        << num_of_sensor_);
    }
    if (!nh_p_.param("serial_port", serial_port_, serial_port_)) {
        ROS_WARN_STREAM("serial_port is not set! Use default " << serial_port_);
    }
    if (!nh_p_.param("serial_baud_rate", serial_baud_rate_,
                     serial_baud_rate_)) {
        ROS_WARN_STREAM("serial_baud_rate is not set! Use default "
                        << serial_baud_rate_);
    }
}

void SensorsManager::startSensors()
{
    for (const auto &sonar : sonars_ | boost::adaptors::indexed(0)) {
        sonar.value().setIndex(sonar.index() + 1);
    }
    std::for_each(sonars_.begin(), (sonars_.begin() + num_of_sensor_),
                  [](auto &sonar) {
                      if (sonar.start()) {
                          sonar.getSonarConfig().printConfig();
                      }
                  });
}

void SensorsManager::initRosPub()
{
    range_sensors_pubs_.resize(num_of_sensor_);
    for (const auto &range_sensor_pub :
         range_sensors_pubs_ | boost::adaptors::indexed(0)) {
        std::string topic_name = "sonar_range_" + range_sensor_pub.index();
        range_sensor_pub.value() =
                nh_p_.advertise<sensor_msgs::Range>(topic_name, 10);
    }
}

void SensorsManager::timerPubCallBack(const ros::TimerEvent &)
{
    std_msgs::Float64 msg;
    std::for_each(sonars_.begin(), (sonars_.begin() + num_of_sensor_),
                  [&](auto &sonar) {
                      sonar.getDistance(ks114_detection_mode_, msg.data);
                      sonars_pub_.publish(msg);
                  });

    sensor_msgs::Range range_msg;
    range_msg.range = msg.data;
    range_msg.header.stamp = ros::Time::now();
    range_msg.radiation_type = range_msg.ULTRASOUND;
    range_msg.min_range =
            (ks114_detection_mode_ == ks114_sonar::DetectionMode::Far) ? 0.03
                                                                       : 0.01;
    range_msg.max_range =
            (ks114_detection_mode_ == ks114_sonar::DetectionMode::Far) ? 1.1
                                                                       : 5.6;
    for (const auto &range_sensor_pub :
         range_sensors_pubs_ | boost::adaptors::indexed(0)) {
        range_sensor_pub.value().publish(range_msg);
    }
}
