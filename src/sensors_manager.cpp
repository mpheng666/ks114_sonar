#include "sensors_manager/sensors_manager.hpp"

using namespace sensors_manager;

SensorsManager::SensorsManager(ros::NodeHandle &nh)
    : nh_p_(nh),
      sonars_pub_(nh_p_.advertise<std_msgs::Float64>("sonars_data", 10)),
      pub_timer_(nh_p_.createTimer(
              ros::Duration(0.05), &SensorsManager::timerCallBack, this))

{
}

SensorsManager::~SensorsManager() {}

void SensorsManager::start()
{
    startSensors();
    loadParams();
    ros::Rate rate = LOOP_RATE_;
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
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

void SensorsManager::timerCallBack(const ros::TimerEvent &)
{
    std_msgs::Float64 msg;
    std::for_each(sonars_.begin(), (sonars_.begin() + num_of_sensor_),
                  [&](auto &sonar) {
                      sonar.getDistance(ks114_detection_mode_, msg.data);
                      sonars_pub_.publish(msg);
                  });
}
