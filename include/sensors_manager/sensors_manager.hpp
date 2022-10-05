#ifndef SENSORS_MANAGER_HPP_
#define SENSORS_MANAGER_HPP_

#include "ks114_sonar/ks114_sonar.hpp"
#include "signal_filter/signal_filter.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <boost/range/adaptors.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace sensors_manager {
static constexpr std::array<std::string_view, 8> ROBOT_BODY{
        "FRONT_RIGHT", "FRONT_LEFT", "LEFT_FRONT", "LEFT_SIDE",
        "BACK_LEFT",   "BACK_RIGHT", "RIGHT_SIDE", "RIGHT_FRONT"};

//                 FRONT
//                _2___1_
//             3 |       | 8
//               |       |
//             4 |       | 7
//                -------
//               5      6
//                 BACK
//           sonar_index_1 = 0XD0;
//           sonar_index_2 = 0XD2;
//           array[0] = sonar_index_1
//           array[1] = sonar_index_2

class SensorsManager {
public:
    explicit SensorsManager(ros::NodeHandle &nh);
    ~SensorsManager();
    void start();

private:
    // ROS related
    ros::NodeHandle nh_p_;
    ros::Publisher sonars_pub_;
    ros::Publisher sonars_filtered_pub_;
    std::vector<ros::Publisher> range_sensors_pubs_;
    ros::SteadyTimer get_data_stimer_;
    ros::Timer pub_timer_;
    static constexpr double LOOP_RATE_{20.0};

    // remapper
    static constexpr int MIN_NUMBER_OF_SONAR_{1};
    static constexpr int MAX_NUMBER_OF_SONAR_{20};
    const std::vector<int> DEFAULT_REMAPPER_{1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int> sonar_remapper_{1, 2, 3, 4, 5, 6, 7, 8};
    int num_of_sonar_{8};

    // serial
    std::string serial_port_{"/dev/ttyUSB0"};
    int serial_baud_rate_{115200};

    // sonar config
    int detection_mode_{0};
    ks114_sonar::DetectionMode ks114_detection_mode_{
            ks114_sonar::DetectionMode::Fast};
    std::array<ks114_sonar::Ks114Sonar, 20> sonars_;
    std::vector<double> sonars_data_raw_;
    std::vector<double> sonars_data_filtered_;
    std::vector<double> sonars_data_filtered_prev_;
    std::vector<ks114_sonar::SonarState> sonars_state_{};

    void loadParams();
    void startSensors();
    void initRosPub();
    void timerGetDataSteadyCallBack(const ros::SteadyTimerEvent &);
    void timerPubCallBack(const ros::TimerEvent &);
};
} // namespace sensors_manager

#endif