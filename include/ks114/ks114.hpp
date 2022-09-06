#ifndef KS114_SONAR_KS114_DRIVER_HPP
#define KS114_SONAR_KS114_DRIVER_HPP

// #define DEBUG_

#include <serial/serial.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

namespace ks114_ns
{
    class SonarKs114
    {
    public:
        SonarKs114(std::string, std::string);
        SonarKs114();
        ~SonarKs114();
        void startProcess();

    private:
        // ros
        ros::NodeHandle private_nh_;
        ros::NodeHandle relative_nh_;
        ros::Publisher ks114_sonar_raw_pub_;
        ros::Publisher ks114_sonar_auxi_pub_;
        std_msgs::Float32MultiArray ks114_sonar_raw_msg_;
        std_msgs::Float32MultiArray ks114_sonar_auxi_msg_;
        static constexpr double LOOP_RATE {25};

        // Serial port
        enum PortState
        {
            PortUnopened,
            PortOpened,
            PortErrored
        };

        PortState _currState = PortUnopened;
        static constexpr char DEFAULT_PORT[] {"/dev/ttyUSB0"};
        static constexpr int DEFAULT_BAUDRATE {115200};
        static constexpr uint32_t PORT_TIMEOUT_MS {1000};
        std::string port_;
        int baud_rate_;
        serial::Serial ser_;

        // Sonar
        static constexpr uint8_t sensor_address[20] 
        {
            0XD0, 0XD2, 0XD4, 0XD6, 0XD8, 0XDA, 0XDC, 0XDE, 
            0XE0, 0XE2, 0XE4, 0XE6, 0XE8, 0XEA, 0XEC, 0XEE, 
            0XF8, 0XFA, 0XFC, 0XFE
        };
        static constexpr int DEFAULT_NUM {8};
        int num_of_sonar_;
        bool check_flag_ {false};
        std::vector<float> ks114_distance_data_;
        static constexpr double range_min_ {3.0};
        static constexpr double range_max_ {105.0};
        static constexpr double range_error_ {600.0};

        // Auxi
        static constexpr int auxi_size_ {8};
        static constexpr double auxi_ignored_data_ {5.0};

        void loadParam();
        bool openSerial(const char* port, const int baudrate);
        void getSensorInfo(uint8_t);
        void checkSensorInfo(int);
        void sendReadCmd(uint8_t);
        float readSensorValue(int);
        void pubSensorData();
        void changeSensorAddress();
        float filterSensorData(float&);

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
