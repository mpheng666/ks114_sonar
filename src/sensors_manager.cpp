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
              ros::Duration(0.1), &SensorsManager::timerPubCallBack, this)),
      updateSonarConfig_(boost::bind(
              &SensorsManager::dynamicReconfigCallBack, this, _1, _2))

{
}

SensorsManager::~SensorsManager() {}

void SensorsManager::start()
{
    loadParams();
    startSensors();
    initRosPub();
    sonar_config_server_.setCallback(updateSonarConfig_);
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
        num_of_sonar_ = sonar_remapper_.size();
        sonars_data_raw_.resize(num_of_sonar_);
        sonars_state_.resize(num_of_sonar_);
        sonars_data_filtered_.resize(num_of_sonar_);
        sonars_data_filtered_prev_.resize(num_of_sonar_);
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
    if (!nh_p_.param("use_threshold_filter", use_threshold_filter_,
                     use_threshold_filter_)) {
        ROS_WARN_STREAM("use_threshold_filter is not set! Use default "
                        << use_threshold_filter_);
    }
    if (!nh_p_.param("use_low_pass_filter", use_low_pass_filter_,
                     use_low_pass_filter_)) {
        ROS_WARN_STREAM("use_low_pass_filter is not set! Use default "
                        << use_low_pass_filter_);
    }
    if (!nh_p_.param("low_pass_gain", low_pass_gain_, low_pass_gain_)) {
        ROS_WARN_STREAM("low_pass_gain is not set! Use default "
                        << low_pass_gain_);
    }
}

void SensorsManager::startSensors()
{
    for (const auto &remapper : sonar_remapper_ | boost::adaptors::indexed(1)) {
        sonars_.at(remapper.value()).setIndex(remapper.value());
        sonars_.at(remapper.value()).setSerialPort(serial_port_);
        sonars_.at(remapper.value()).setSerialBaudRate(serial_baud_rate_);
        if (sonars_.at(remapper.value()).start()) {
            sonars_.at(remapper.value()).getSonarConfig().printConfig();
            sonars_state_.at(remapper.index() - 1) =
                    ks114_sonar::SonarState::Started;
        }
        else {
            sonars_state_.at(remapper.index() - 1) =
                    ks114_sonar::SonarState::Error;
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
    if (use_low_pass_filter_ || use_threshold_filter_) {
        sonars_filtered_pub_ = nh_p_.advertise<std_msgs::Float64MultiArray>(
                "sonars_filtered_data", 10);
    }
}

void SensorsManager::timerGetDataSteadyCallBack(const ros::SteadyTimerEvent &)
{
    min_reading = (ks114_detection_mode_ == ks114_sonar::DetectionMode::Fast)
                          ? 0.03
                          : 0.01;
    max_reading = (ks114_detection_mode_ == ks114_sonar::DetectionMode::Fast)
                          ? 1.1
                          : 5.6;

    for (auto i = 0; i < num_of_sonar_; ++i) {
        if (sonars_state_.at(i) == ks114_sonar::SonarState::Started) {
            if (sonars_.at(sonar_remapper_.at(i))
                        .getDistance(ks114_detection_mode_,
                                     sonars_data_raw_.at(i))) {
                if (use_threshold_filter_) {
                    sonars_data_filtered_.at(i) =
                            signal_filter::thresholdFilter(
                                    min_reading, max_reading,
                                    sonars_data_raw_.at(i));
                }
                else {
                    sonars_data_filtered_.at(i) = sonars_data_raw_.at(i);
                }
                if (use_low_pass_filter_) {
                    sonars_data_filtered_.at(i) = signal_filter::lowPassFilter(
                            low_pass_gain_, sonars_data_filtered_.at(i),
                            sonars_data_filtered_prev_.at(i));
                }
            }
            else {
                sonars_state_.at(i) = ks114_sonar::SonarState::Error;
                ROS_WARN_STREAM("SONAR "
                                << sonar_remapper_.at(i)
                                << " DISCONNECTED FOR THE FIRST TIME!");
            }
        }
        else {
            // ROS_WARN_STREAM("Attempting to reconnect sonar "
            //                 << sonar_remapper_.at(i));
            if (sonars_.at(sonar_remapper_.at(i)).start()) {
                sonars_.at(sonar_remapper_.at(i))
                        .getSonarConfig()
                        .printConfig();
                ROS_WARN_STREAM("SONAR " << sonar_remapper_.at(i)
                                         << " RECONNECTED!");
                sonars_state_.at(i) = ks114_sonar::SonarState::Started;
            }
            else {
                ROS_WARN_STREAM("SONAR " << sonar_remapper_.at(i)
                                         << " DISCONNECTED CONTINUOUSLY! ");
                sonars_state_.at(i) = ks114_sonar::SonarState::Error;

                sonars_data_raw_.at(i) = ks114_sonar::DATA_GIVEN_IF_ERROR;
            }
        }
    }
}

void SensorsManager::timerPubCallBack(const ros::TimerEvent &)
{
    std_msgs::Float64MultiArray raw_msgs;
    std_msgs::Float64MultiArray filtered_msgs;
    sensor_msgs::Range range_msg;
    range_msg.radiation_type = range_msg.ULTRASOUND;
    range_msg.min_range = min_reading;
    range_msg.max_range = max_reading;
    for (const auto &sonar_data :
         sonars_data_raw_ | boost::adaptors::indexed(0)) {

        raw_msgs.data.emplace_back(sonar_data.value());

        range_msg.header.stamp = ros::Time::now();
        range_msg.range = sonar_data.value();
        range_sensors_pubs_.at(sonar_data.index()).publish(range_msg);
    }
    sonars_pub_.publish(raw_msgs);

    if (use_low_pass_filter_ || use_threshold_filter_) {
        filtered_msgs.data = sonars_data_filtered_;
        sonars_filtered_pub_.publish(filtered_msgs);
        sonars_data_filtered_prev_ = std::move(filtered_msgs.data);
    }
}

void SensorsManager::dynamicReconfigCallBack(ks114_sonar::sonarConfig &config,
                                             uint32_t level)
{
    ks114_detection_mode_ =
            static_cast<ks114_sonar::DetectionMode>(config.detection_mode);
    use_threshold_filter_ = config.use_threshold_filter;
    use_low_pass_filter_ = config.use_low_pass_filter;
    low_pass_gain_ = config.low_pass_gain;
}
