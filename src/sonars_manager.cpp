#include "ks114_sonar/sonars_manager.hpp"

namespace ks114_sonar
{
    SonarsManager::SonarsManager(ros::NodeHandle& nh)
        : nh_p_(nh)
        , raw_values_pub_(
              nh_p_.advertise<std_msgs::Float64MultiArray>("raw_readings", 10))
        , filtered_values_pub_(
              nh_p_.advertise<std_msgs::Float64MultiArray>("filtered_readings", 10))
        , comms_handler_()
    {
        ROS_INFO_STREAM("Started sonars manager!");
        loadParams();
    }

    void SonarsManager::start()
    {
        while (ros::ok())
        {
            std::vector<double> raw_readings(sonars_remappper_.size());
            for (int i = 0; i < sonars_remappper_.size(); ++i)
            {
                sonars_reader_.at(i).start();
                if (auto val = sonars_reader_.at(i).getDistance(
                        static_cast<DetectionMode>(detection_mode_)))
                {
                    raw_readings.at(i) = val.value();
                }
                else
                {
                    raw_readings.at(i) = ERROR_READING_;
                }
            }
            pubReadings(raw_readings);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    }

    void SonarsManager::loadParams()
    {
        if (!nh_p_.param("serial_port", serial_port_name_, serial_port_name_))
        {
            ROS_WARN_STREAM("serial_port is not set! Use default " << serial_port_name_);
        }
        else
        {
            comms_handler_.setPort(serial_port_name_, BAUDRATE_);
        }
        if (!nh_p_.param("serial_timeout_ms", comms_timeout_ms_, comms_timeout_ms_))
        {
            ROS_WARN_STREAM("serial_timeout_ms is not set! Use default "
                            << comms_timeout_ms_);
        }
        else
        {
            comms_handler_.setSerialTimeOut(comms_timeout_ms_);
        }
        if (!nh_p_.param("detection_mode", detection_mode_, detection_mode_))
        {
            ROS_WARN_STREAM("detection_mode is not set! Use default "
                            << detection_mode_);
        }
        if (!nh_p_.param(
                "use_low_pass_filter", use_low_pass_filter_, use_low_pass_filter_))
        {
            ROS_WARN_STREAM("use_low_pass_filter is not set! Use default "
                            << use_low_pass_filter_);
        }
        if (!nh_p_.param("low_pass_gain", low_pass_gain_, low_pass_gain_))
        {
            ROS_WARN_STREAM("low_pass_gain is not set! Use default " << low_pass_gain_);
        }

        if (nh_p_.param("sonar_remapper", sonars_remappper_, sonars_remappper_))
        {
            for (int i = 0; i < sonars_remappper_.size(); ++i)
            {
                sonars_.emplace_back(sonars_remappper_.at(i));
            }
            for (auto& sonar : sonars_)
            {
                sonars_reader_.emplace_back(sonar, comms_handler_);
            }
            previous_filtered_values_.resize(sonars_remappper_.size());
        }
        else
        {
            ROS_WARN_STREAM("sonar_remapper is not set!");
        }
    }

    void SonarsManager::pubReadings(const std::vector<double>& readings)
    {
        std_msgs::Float64MultiArray msg;
        msg.data = readings;
        raw_values_pub_.publish(msg);

        std_msgs::Float64MultiArray filtered_msg;
        std::transform(previous_filtered_values_.begin(),
                       previous_filtered_values_.end(),
                       readings.begin(),
                       std::back_inserter(filtered_msg.data),
                       [&](auto prev_val, auto curr_val) {
                           return (1 - low_pass_gain_) * prev_val +
                                  low_pass_gain_ * curr_val;
                       });
        previous_filtered_values_ = filtered_msg.data;
        filtered_values_pub_.publish(filtered_msg);
    }

}  // namespace ks114_sonar