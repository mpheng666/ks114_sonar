#include "ks114/ks114.hpp"

namespace ks114_ns
{

    SonarKs114::SonarKs114(ros::NodeHandle& nh)
        : 
        private_nh_(nh),
        ks114_sonar_data_pub_(nh.advertise<std_msgs::Float32MultiArray>("sonar_data", 10)),
        ks114_sonar_auxi_pub_(nh.advertise<std_msgs::Float32MultiArray>("sonar_auxi", 10))
        // timer_send_detection(nh.createTimer(ros::Duration(1.0/detection_rate_), &SonarKs114::readSonar, this))
    {
    }

    SonarKs114::SonarKs114()
    {
    }

    SonarKs114::~SonarKs114()
    {
    }

    void SonarKs114::loadParam()
    {
        if(!private_nh_.getParam("detection_mode", detection_mode_param_))
        {
            ROS_WARN("Sonar detection mode is not set. Using %u as default", detection_mode_param_);
            detection_mode_ = static_cast<DetectionMode>(detection_mode_param_);
        }
        if (!private_nh_.getParam("num_of_sonar", num_of_sonar_))
        {
            ROS_WARN("Number of sonar is not set. Using %u as default", num_of_sonar_);
        }
        if (!private_nh_.getParam("/ks114_sonar/serial_port", port_))
        {
            ROS_WARN("Sonar serial port is not set. Using %s as default", port_.c_str());
        }
        if (!private_nh_.getParam("/ks114_sonar/serial_baud_rate", baud_rate_))
        {
            ROS_WARN("Sonar baud rate is not set. Using %u as default", baud_rate_);
        }
    }

    bool SonarKs114::openSerial(const std::string port, const int baudrate)
    {

        if (port_state_ == PortState::PortUnopened)
        {
            try
            {
                ser_.setPort(port);
                ser_.setBaudrate(baudrate);
                auto timeout = serial::Timeout::simpleTimeout(PORT_TIMEOUT_MS);
                ser_.setTimeout(timeout);
                ser_.open();
            }
            catch (const std::exception &exc)
            {
                ROS_ERROR_STREAM("Unable to open port!" << exc.what());
                port_state_ = PortState::PortErrored;
                return false;
            }

            if (ser_.isOpen())
            {
                ROS_INFO_STREAM("Serial Port initialized");
                port_state_ = PortState::PortOpened;
                return true;
            }
        }

        return false;
    }

    void SonarKs114::startProcess()
    {
        this->loadParam();
        ros::Rate r(LOOP_RATE);

        if(this->openSerial(port_.c_str(), baud_rate_))
        {
            this->initSensorsInfo();
        }

        while (ros::ok())
        {
            if (port_state_ == PortState::PortOpened)
            {
                std::vector<double> sensors_output;
                if(this->readSensorsValue(sensors_output))
                {
                    #ifdef DEBUG
                        for(const auto& o : sensors_output)
                        {
                            ROS_INFO_STREAM("output: " << o << " ");
                        }
                    #endif
                    this->pubSensorsData(sensors_output);
                }
            } 
            else if (port_state_ == PortState::PortUnopened || port_state_ == PortState::PortErrored)
            {
                this->openSerial(port_.c_str(), baud_rate_);
            }
            r.sleep();
            ros::spinOnce();
        }
    }

    void SonarKs114::initSensorsInfo()
    {
        int index = 0;
        std::for_each(
            SENSOR_ADDRESS.begin(), 
            SENSOR_ADDRESS.begin()+num_of_sonar_,
            [&](auto& address)
            {
                ser_.flushInput();
                const uint8_t command[3] {address, 0x02, GET_INFO_COMMAND};
                ser_.write(command, 3);

                ros::Rate(LOOP_RATE).sleep();

                std::vector<uint8_t> byte_received;

                if(ser_.available())
                {
                    ser_.read(byte_received, 22);

                    #ifdef DEBUG
                        for(const auto& b : byte_received)
                        {
                            ROS_INFO("b: %02X", b);
                        }
                    #endif
   
                    if(byte_received.at(5) == address)
                    {
                        ROS_INFO("Sonar %d is CONNECTED!", index);
                    }
                }
                else
                {
                    ROS_ERROR("Sonar %d is NOT CONNECTED OR CONFIGURED!", index);
                }
                ++index;
                byte_received.clear();
            }
        );
    }

    bool SonarKs114::readSensorsValue(std::vector<double>& output)
    {
        int index = 0;
        std::for_each(
            SENSOR_ADDRESS.begin(), 
            SENSOR_ADDRESS.begin()+num_of_sonar_,
            [&](auto& address)
            {
                ser_.flushInput();
                const uint8_t command[3] {address, 0x02, DETECT_FAST_COMMAND};
                ser_.write(command, 3);

                ros::Rate(LOOP_RATE).sleep();

                std::vector<uint8_t> byte_received;

                if(ser_.available())
                {
                    ser_.read(byte_received, 2);
                    auto value_in_byte = static_cast<uint16_t>(byte_received[0] << 8 | byte_received[1]);
                    auto value_in_m = static_cast<float>(value_in_byte)/5800.0;
                    output.push_back(this->filterSensorsData(value_in_m, detection_mode_));

                    #ifdef DEBUG
                        for(const auto& b : byte_received)
                        {
                            ROS_INFO("b: %02X", b);
                        }
                    #endif
   
                }
                else
                {
                    ROS_ERROR("Sonar %d is NOT CONNECTED!", index);
                }
                ++index;
                byte_received.clear();
            }
        );
        if(output.size() > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    double SonarKs114::filterSensorsData(const double input, const DetectionMode mode)
    {
        if(mode == DetectionMode::FastDetection)
        {
            if(input > RANGE_MIN && input < RANGE_MAX_FAST)
                return input;
            else 
                return RANGE_ERROR;
        }
        else if (mode == DetectionMode::NormalDetection)
        {
            if(input > RANGE_MIN && input < RANGE_MAX_NORMAL)
                return input;
            else 
                return RANGE_ERROR;
        }
    }

    void SonarKs114::pubSensorsData(const std::vector<double>& output)
    {
        // assert(output.size() == num_of_sonar_);
        std_msgs::Float64MultiArray sonar_msg;
        sonar_msg.data.resize(num_of_sonar_, 0);
        // std::copy(ks114_distance_data_.begin(), ks114_distance_data_.end(), ks114_sonar_raw_msg_.data.begin());
        // sonar_msg.data = std::move(output);
        std::copy(output.begin(), output.end(), sonar_msg.data.begin());
        ks114_sonar_data_pub_.publish(sonar_msg);

        // ks114_sonar_auxi_msg_.data[0] = ks114_distance_data_[0];
        // ks114_sonar_auxi_msg_.data[1] = ks114_distance_data_[1];
        // ks114_sonar_auxi_msg_.data[2] = ks114_distance_data_[2];
        // ks114_sonar_auxi_msg_.data[3] = ks114_distance_data_[3];
        // ks114_sonar_auxi_msg_.data[4] = ks114_distance_data_[4];
        // ks114_sonar_auxi_msg_.data[5] = ks114_distance_data_[5];
        // ks114_sonar_auxi_msg_.data[6] = ks114_distance_data_[6];
        // ks114_sonar_auxi_msg_.data[7] = ks114_distance_data_[7];
        // ks114_sonar_auxi_pub_.publish(ks114_sonar_auxi_msg_);
    }

}; // ks114_ns