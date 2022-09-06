#include "ks114/ks114.hpp"

namespace ks114_ns
{

    SonarKs114::SonarKs114(ros::NodeHandle& nh)
        : ks114_sonar_data_pub_(nh.advertise<std_msgs::Float32MultiArray>("sonar_data", 10)),
          ks114_sonar_auxi_pub_(nh.advertise<std_msgs::Float32MultiArray>("sonar_auxi", 10))
    {
        ks114_sonar_raw_msg_.data.resize(num_of_sonar_, 0);
        ks114_sonar_auxi_msg_.data.resize(auxi_size_, 0);
    }

    SonarKs114::SonarKs114()
    {
    }

    SonarKs114::~SonarKs114()
    {
    }

    void SonarKs114::loadParam()
    {
        // Add detect mode param
        if (!ros::param::param<int>("/ks114_sonar/num_of_sonar", num_of_sonar_, DEFAULT_NUM))
        {
            ROS_WARN("Number of sonar is not set. Using %u as default", DEFAULT_NUM);
        }
        if (!ros::param::param<std::string>("/ks114_sonar/serial_port", port_, DEFAULT_PORT))
        {
            ROS_WARN("Sonar serial port is not set. Using %s as default", DEFAULT_PORT);
        }
        if (!ros::param::param<int>("/ks114_sonar/serial_baud_rate", baud_rate_, DEFAULT_BAUDRATE))
        {
            ROS_WARN("Sonar baud rate is not set. Using %u as default", DEFAULT_BAUDRATE);
        }
    }

    bool SonarKs114::openSerial(const char* port, const int baudrate)
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
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("Unable to open port ");
                port_state_ = PortState::PortErrored;
                return false;
            }

            if (ser_.isOpen())
            {
                ROS_INFO_STREAM("Serial Port initialized");
                port_state_ = PortState::PortOpened;
                return true;
            }
            else
            {
                port_state_ = PortState::PortUnopened;
                return false;
            }
        }
    }

    void SonarKs114::startProcess()
    {
        this->loadParam();
        ros::Rate r(LOOP_RATE);
        ros::Rate r2(5);
        uint32_t distance_;

        while (ros::ok())
        {
            if (port_state_ == PortState::PortOpened && check_flag_ == true)
            {
                for (int i = 0; i < num_of_sonar_; i++)
                {
                    this->sendReadCmd(sensor_address[i]);
                    r.sleep();
                    ks114_distance_data_.push_back(this->readSensorValue(i));
                }
                this->pubSensorData();
                ks114_distance_data_.clear();
            } 
            else if (port_state_ == PortState::PortUnopened)
            {
                this->openSerial(port_.c_str(), baud_rate_);
            }
            else if (port_state_ == PortState::PortOpened && check_flag_ == false)
            {
                for (int i = 0; i < num_of_sonar_; i++)
                {
                    this->getSensorInfo(sensor_address[i]);
                    r.sleep();
                    this->checkSensorInfo(i);
                }
                check_flag_ = true;
            }
            ros::spinOnce();
        }
    }

    void SonarKs114::getSensorInfo(uint8_t address_)
    {
        uint8_t info_cmd_single[3] = {address_, 0x02, 0x99};
        ser_.flushInput();
        ser_.write(info_cmd_single, 3);
    }

    void SonarKs114::checkSensorInfo(int index)
    {
        std::vector<uint8_t> byte_received;

        if (ser_.available())
        {
            ser_.read(byte_received, 22);
            if(byte_received[5] == sensor_address[index])
            ROS_INFO("Sonar %d is CONNECTED!", index);
        }
        else
        {
            ROS_ERROR("Sonar %d is NOT CONNECTED OR CONFIGURED!", index);
        }
        byte_received.clear();
    }

    void SonarKs114::sendReadCmd(uint8_t address_)
    {
        // B0 for high speed read (0.01m ~ 1.1m)
        uint8_t detect_cmd_single[3] = {address_, 0x02, 0x0F}; 
        // 0F for normal speed read (0.03m ~ 5.6m)
        // uint8_t detect_cmd_single[3] = {address_, 0x02, 0xB0}; 
        ser_.flushInput();
        ser_.write(detect_cmd_single, 3);
    }

    float SonarKs114::readSensorValue(int index)
    {
        std::vector<uint8_t> byte_received;
        uint16_t distance_uint16_raw_ = 0;
        float distance_raw_ = 0.0;
        float distance_filtered_ = 0.0;

        if (ser_.available())
        {
            ser_.read(byte_received, 2);
            distance_uint16_raw_ = (uint16_t(byte_received[0] << 8 | byte_received[1]));
            distance_raw_ = float(distance_uint16_raw_)/58.0;

            #ifdef DEBUG_
                if (index == 0)
                {
                    ROS_INFO("serial read: %02X %02X", byte_received[0], byte_received[1]);
                    ROS_INFO("distance %d : %f \n", index, distance_raw_);
                }
            #endif
            
        }
        else
        {
            ROS_ERROR("Sonar %d is NOT CONNECTED!", index);
        }
        byte_received.clear();
        distance_filtered_ = filterSensorData(distance_raw_);

        return (distance_filtered_/100.0);
    }

    float SonarKs114::filterSensorData(float &distance_unfiltered_)
    {
        if(distance_unfiltered_ > this->range_min_ && distance_unfiltered_ < this->range_max_)
            return distance_unfiltered_;
        else 
            return this->range_error_;
    }

    void SonarKs114::pubSensorData()
    {
        std::copy(ks114_distance_data_.begin(), ks114_distance_data_.end(), ks114_sonar_raw_msg_.data.begin());
        ks114_sonar_auxi_msg_.data[0] = ks114_distance_data_[0];
        ks114_sonar_auxi_msg_.data[1] = ks114_distance_data_[1];
        ks114_sonar_auxi_msg_.data[2] = ks114_distance_data_[2];
        ks114_sonar_auxi_msg_.data[3] = ks114_distance_data_[3];
        ks114_sonar_auxi_msg_.data[4] = ks114_distance_data_[4];
        ks114_sonar_auxi_msg_.data[5] = ks114_distance_data_[5];
        ks114_sonar_auxi_msg_.data[6] = ks114_distance_data_[6];
        ks114_sonar_auxi_msg_.data[7] = ks114_distance_data_[7];
        ks114_sonar_raw_pub_.publish(ks114_sonar_raw_msg_);
        ks114_sonar_auxi_pub_.publish(ks114_sonar_auxi_msg_);
    }

}; // ks114_ns