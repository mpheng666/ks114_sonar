#ifndef KS114_SONAR_SENSOR_READER_HPP_
#define KS114_SONAR_SENSOR_READER_HPP_

#include "ks114_sonar/comms_handler.hpp"
#include "ks114_sonar/ks114_sonar.hpp"

namespace ks114_sonar
{
    class SensorReader
    {
        public:
        SensorReader(CommsHandler& comms, Ks114Sonar& sonar);
        ~SensorReader();
        void read(double read_frequency_ms_, bool oneshot);
        void start();

        private:
        CommsHandler& comms_handler_;
        Ks114Sonar& sonar_;

        boost::asio::io_context ioc_;
        boost::asio::steady_timer read_timer_;

        double distance_m_ {0.0};
        double read_frequency_ms_ {500.0};

        bool establishConnection();
        void readSensor(const boost::system::error_code& ec);
    };

}  // namespace ks114_sonar

#endif
