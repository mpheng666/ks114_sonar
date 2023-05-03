#include "ks114_sonar/sensor_reader.hpp"

#include <chrono>

namespace ks114_sonar
{
    SensorReader::SensorReader(CommsHandler& comms, Ks114Sonar& sonar)
        : comms_handler_(comms)
        , sonar_(sonar)
        , read_timer_(ioc_)
    {
    }

    SensorReader::~SensorReader() { ioc_.stop(); }

    bool SensorReader::establishConnection()
    {
        if (comms_handler_.start())
        {
            return true;
        }
        return false;
    }

    void SensorReader::start()
    {
        if (establishConnection())
        {
            read_timer_.expires_after(
                boost::asio::chrono::milliseconds(static_cast<int>(read_frequency_ms_)));
            read_timer_.async_wait(boost::bind(
                &SensorReader::readSensor, this, boost::asio::placeholders::error));
            ioc_.run();
        }
    }

    void SensorReader::read(double read_frequency_ms, bool oneshot)
    {
        if (comms_handler_.start())
        {
            read_frequency_ms_ = read_frequency_ms;
            read_timer_.expires_after(
                boost::asio::chrono::milliseconds(static_cast<int>(read_frequency_ms_)));
            read_timer_.async_wait(boost::bind(
                &SensorReader::readSensor, this, boost::asio::placeholders::error));
            ioc_.run();
        }
    }

    void SensorReader::readSensor(const boost::system::error_code& ec)
    {
        if (ec)
        {
            std::cerr << ec.message() << "\n";
        }
        else
        {
            auto sense_command = sonar_.getSenseCommand(sonar_.getSonarConfig().address,
                                                        DetectionMode::Fast);
            comms_handler_.write(sense_command, 100);
            auto result = comms_handler_.getReadBuffer();
            for (const auto& res : result)
            {
                std::cout << "result: " << unsigned(res) << "\n";
            }
        }

        read_timer_.expires_at(
            read_timer_.expiry() +
            boost::asio::chrono::milliseconds(static_cast<int>(read_frequency_ms_)));

        read_timer_.async_wait(boost::bind(
            &SensorReader::readSensor, this, boost::asio::placeholders::error));
    }

}  // namespace ks114_sonar