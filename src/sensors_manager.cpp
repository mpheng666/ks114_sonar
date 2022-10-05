#include "sensors_manager/sensors_manager.hpp"

using namespace sensors_manager;

SensorsManager::SensorsManager(ros::NodeHandle &nh)
    : nh_p_(nh), sonars_pub_(nh_p_.advertise<std_msgs::Float64MultiArray>(
                         "sonars_data", 10)),
      get_data_stimer_(nh_p_.createSteadyTimer(
              ros::WallDuration(0.5),
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
            return (n < MIN_NUMBER_OF_SONAR_ || n > MAX_NUMBER_OF_SONAR_)
                           ? true
                           : false;
        };
        if (sonar_remapper_.size() < MIN_NUMBER_OF_SONAR_ ||
            sonar_remapper_.size() > MAX_NUMBER_OF_SONAR_) {
            sonar_remapper_ = DEFAULT_REMAPPER_;
            ROS_WARN_STREAM("sonar_remapper is wrong-sized! Use default: \n1 2 "
                            "3 4 5 6 7 8");
        }
        else if (std::any_of(sonar_remapper_.begin(), sonar_remapper_.end(),
                             checkElementsOutOfRange)) {
            sonar_remapper_ = DEFAULT_REMAPPER_;
            ROS_WARN_STREAM(
                    "sonar_remapper is out-of-ranged! Use default: \n1 2 "
                    "3 4 5 6 7 8");
        }
        else if (std::unique(sonar_remapper_sorted.begin(),
                             sonar_remapper_sorted.end()) !=
                 sonar_remapper_sorted.end()) {
            sonar_remapper_ = DEFAULT_REMAPPER_;
            ROS_WARN_STREAM("sonar_remapper is duplicated! Use default: \n1 2 "
                            "3 4 5 6 7 8");
        }
        else {
            num_of_sonar_ = sonar_remapper_.size();
            sonars_data_raw_.resize(num_of_sonar_);
        }
    }
    if (!nh_p_.param("detection_mode", detection_mode_, detection_mode_)) {
        ROS_WARN_STREAM("detection_mode is not set! Use default "
                        << detection_mode_);
    }
    else {
        ks114_detection_mode_ =
                (detection_mode_ == 0)
                        ? ks114_sonar::DetectionMode::Far
                        : (detection_mode_ == 1)
                                  ? ks114_sonar::DetectionMode::Fast
                                  : ks114_sonar::DetectionMode::Fast;
    }
}

void SensorsManager::startSensors()
{
    for (const auto &sonar : sonars_ | boost::adaptors::indexed(1)) {
        sonar.value().setIndex(sonar.index());
        sonar.value().setSerialPort(serial_port_);
        sonar.value().setSerialBaudRate(serial_baud_rate_);
        if (sonar.value().start()) {
            sonar.value().getSonarConfig().printConfig();
        }
    }
}

void SensorsManager::initRosPub()
{
    for (auto i = 0; i < num_of_sonar_; ++i) {
        std::string topic_name =
                "sonar_" + std::to_string(sonar_remapper_.at(i));
        range_sensors_pubs_.emplace_back(
                nh_p_.advertise<sensor_msgs::Range>(topic_name, 10));
        ;
    }
}

void SensorsManager::timerGetDataSteadyCallBack(const ros::SteadyTimerEvent &)
{
    for (auto i = 0; i < num_of_sonar_; ++i) {
        sonars_.at(sonar_remapper_.at(i) - 1)
                .getDistance(ks114_detection_mode_, sonars_data_raw_.at(i));
    }
}

void SensorsManager::timerPubCallBack(const ros::TimerEvent &)
{
    std_msgs::Float64MultiArray msgs;
    sensor_msgs::Range range_msg;
    range_msg.radiation_type = range_msg.ULTRASOUND;
    range_msg.min_range =
            (ks114_detection_mode_ == ks114_sonar::DetectionMode::Fast) ? 0.03
                                                                        : 0.01;
    range_msg.max_range =
            (ks114_detection_mode_ == ks114_sonar::DetectionMode::Fast) ? 1.1
                                                                        : 5.6;
    for (const auto &sonar_data :
         sonars_data_raw_ | boost::adaptors::indexed(0)) {
        msgs.data.emplace_back(sonar_data.value());
        range_msg.header.stamp = ros::Time::now();
        range_msg.range = sonar_data.value();
        range_sensors_pubs_.at(sonar_data.index()).publish(range_msg);
    }
    sonars_pub_.publish(msgs);
}
