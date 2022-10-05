#include "sensors_manager/sensors_manager.hpp"

using namespace sensors_manager;

SensorsManager::SensorsManager(ros::NodeHandle &nh)
    : nh_p_(nh),
      sonars_pub_(nh_p_.advertise<std_msgs::Float64>("sonars_data", 10)),
      get_data_stimer_(nh_p_.createSteadyTimer(
              ros::WallDuration(0.05),
              &SensorsManager::timerGetDataSteadyCallBack,
              this)),
      pub_timer_(nh_p_.createTimer(
              ros::Duration(0.05), &SensorsManager::timerPubCallBack, this))

{
}

SensorsManager::~SensorsManager() {}

void SensorsManager::start()
{
    loadParams();
    updateVariables();
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
    if (!nh_p_.param("sonar_remapper", sonar_remapper_, sonar_remapper_)) {
        ROS_WARN_STREAM(
                "sonar_remapper is not set! Use default: \n1 2 3 4 5 6 7 8");
    }
    else {
        std::vector<int> sonar_remapper_sorted(sonar_remapper_.size());
        std::partial_sort_copy(sonar_remapper_.begin(), sonar_remapper_.end(),
                               sonar_remapper_sorted.begin(),
                               sonar_remapper_sorted.end());
        auto checkElementsOutOfRange = [](const int &n) {
            return (n < 1 || n > 20) ? true : false;
        };
        if (sonar_remapper_.size() != 8) {
            sonar_remapper_ = {1, 2, 3, 4, 5, 6, 7, 8};
            ROS_WARN_STREAM("sonar_remapper is wrong-sized! Use default: \n1 2 "
                            "3 4 5 6 7 8");
        }
        else if (std::any_of(sonar_remapper_.begin(), sonar_remapper_.end(),
                             checkElementsOutOfRange)) {
            sonar_remapper_ = {1, 2, 3, 4, 5, 6, 7, 8};
            ROS_WARN_STREAM(
                    "sonar_remapper is out-of-ranged! Use default: \n1 2 "
                    "3 4 5 6 7 8");
        }
        else if (std::unique(sonar_remapper_sorted.begin(),
                             sonar_remapper_sorted.end()) !=
                 sonar_remapper_sorted.end()) {
            sonar_remapper_ = {1, 2, 3, 4, 5, 6, 7, 8};
            ROS_WARN_STREAM("sonar_remapper is duplicated! Use default: \n1 2 "
                            "3 4 5 6 7 8");
        }
    }
}

void SensorsManager::updateVariables()
{
    ks114_detection_mode_ =
            (detection_mode_ == 0)
                    ? ks114_sonar::DetectionMode::Far
                    : (detection_mode_ == 1) ? ks114_sonar::DetectionMode::Fast
                                             : ks114_sonar::DetectionMode::Fast;
    data_msgs_.data.resize(num_of_sensor_);
    range_msgs_.resize(num_of_sensor_);
    for (const auto &pos : ROBOT_BODY | boost::adaptors::indexed(0)) {
        sonar_position_map_.emplace(pos.value(),
                                    sonar_remapper_.at(pos.index()));
    }
    // for (const auto &[key, value] : sonar_position_map_) {
    //     std::cout << key << " : " << value << "\n";
    // }
}

void SensorsManager::startSensors()
{
    for (const auto &sonar : sonars_ | boost::adaptors::indexed(0)) {
        sonar.value().setIndex(sonar.index() + 1);
        sonar.value().setSerialPort(serial_port_);
        sonar.value().setSerialBaudRate(serial_baud_rate_);
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

void SensorsManager::timerGetDataSteadyCallBack(const ros::SteadyTimerEvent &)
{
    // std::for_each(sonars_.begin(), (sonars_.begin() + num_of_sensor_),
    //               [&](auto &sonar) {
    //                   sonar.getDistance(ks114_detection_mode_, // TODO );
    //               });
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
