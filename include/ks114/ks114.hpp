#ifndef _KS114_SONAR_KS114_DRIVER_HPP_
#define _KS114_SONAR_KS114_DRIVER_HPP_

// #define DEBUG 

#include <serial/serial.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <sensor_msgs/Range.h>

#include "sensor_filters/sensor_filters.hpp"

namespace ks114_ns
{
    namespace ranges
    {
        template<typename Range, typename Function>
        Function for_each(Range& range, Function f)
        {
            return std::for_each(begin(range), end(range), f);
        }

        template<typename IRange, typename ORange, typename Function>
        Function transform(IRange& input, ORange& output, Function f)
        {
            return std::transform(begin(input), end(input), std::back_inserter(output), f);
        }

        // template<typename IRange, typename ORange, typename Function>
        // Function copy(IRange& input, ORange& output)
        // {
        //     return std::copy(begin(input), end(output), std::back_inserter(output));
        // }
    }

    class SonarKs114
    {
    public:
        explicit SonarKs114(ros::NodeHandle& nh);
        SonarKs114();
        ~SonarKs114();
        void startProcess();

    private:
        // ros
        ros::NodeHandle private_nh_;
        ros::Publisher ks114_sonar_data_pub_;
        ros::Publisher ks114_fitlered_sonar_data_pub_;
        ros::Publisher i2r_auxi_pub_;
        static constexpr double LOOP_RATE {25.0};

        // Serial port
        enum class PortState
        {
            PortUnopened,
            PortOpened,
            PortErrored
        };

        enum class DetectionMode
        {
            NormalDetection,
            FastDetection
        };

        PortState port_state_ = PortState::PortUnopened;
        static constexpr uint32_t PORT_TIMEOUT_MS {1000};
        std::string port_ {"/dev/ttyUSB0"};
        int baud_rate_ {115200};
        serial::Serial ser_;

        // Sonar
        static constexpr std::array<uint8_t, 20> SENSOR_ADDRESS 
        {
            0XD0, 0XD2, 0XD4, 0XD6, 0XD8, 0XDA, 0XDC, 0XDE, 
            0XE0, 0XE2, 0XE4, 0XE6, 0XE8, 0XEA, 0XEC, 0XEE, 
            0XF8, 0XFA, 0XFC, 0XFE
        };
        static constexpr uint8_t GET_INFO_COMMAND {0x99};
        static constexpr uint8_t DETECT_FAST_COMMAND {0x0F};
        static constexpr uint8_t DETECT_NORMAL_COMMAND {0xB0};
        static constexpr int DEFAULT_NUM {8};
        static constexpr double RANGE_MIN {0.03};
        static constexpr double RANGE_MAX_NORMAL {5.95};
        static constexpr double RANGE_MAX_FAST {1.05};
        static constexpr double RANGE_ERROR {404.0};
        int num_of_sonar_ {8};
        std::vector<float> ks114_distance_data_;
        int detection_mode_param_ {0};
        DetectionMode detection_mode_ {DetectionMode::NormalDetection};
        double detection_rate_ {10.0};

        std::map<int, bool> sonars_connection_state;

        void loadParam();
        bool openSerial(const std::string& port, const int baudrate);
        void initSensorsInfo();
        void sendSerialCommand(const uint8_t command[3]);
        void receiveSerialCommand(std::vector<uint8_t>& byte_received, const size_t size);
        bool readSensorsValue(std::vector<double>& output);
        void pubSensorsData(const std::vector<double>& output);
        double filterSensorsData(const double input);
    };
}; //ks114_ns

#endif

/*
        --- List of possible addresses for KS114 ---
        The convention used here is always start from front right middle's index = 0 counter-clockwise to N
        Eg:
              FRONT
             _1___0_
          2 |       | 7
            |       |
          3 |       | 6
             -------
            4      5
               BACK
        sensor_address[0] = 0XD0;
        sensor_address[1] = 0XD2;
*/
