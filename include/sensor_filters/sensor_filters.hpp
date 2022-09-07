#ifndef _SENSOR_FILTERS_HPP_
#define _SENSOR_FILTERS_HPP_

namespace sensor_filters_ns{

    template<typename T>
    class ThresholdFilter{
        public:
            explicit ThresholdFilter::ThresholdFilter(const T min, const T max):
            :
                min_limit_(min),
                max_limit_(max)
            {

            }
            ThresholdFilter::~ThresholdFilter()
            {

            }

            inline T filter(const T& input)
            {
                if(input < min_limit_) return min_limit_;
                if(input > max_limit_) return max_limit_;
                return input;
            }

        private:
            T min_limit_;
            T max_limit_;
    };

    template<typename T>
    class LowPassFilter{
        public:
            explicit LowPassFilter::LowPassFilter(const double gain)
            :
                low_pass_gain_(gain)
            {

            }

            LowPassFilter::~LowPassFilter()
            {

            }

            inline T filter(const T& input, const T& input_previous)
            {
                return low_pass_gain_ * input + (1 - low_pass_gain_) * input_previous;
            }

            void setGain(const double gain)
            {
                low_pass_gain_ = gain;
            }

            T getGain() const
            {
                return low_pass_gain_;
            }

        private:
            double low_pass_gain_ {0.0};
    };

    class MedianFilter{
        public:
            MedianFilter::MedianFilter();
            MedianFilter::~MedianFilter();
            void filter();
    };

    class BandPassFilter{
        public:
            BandPassFilter::BandPassFilter();
            BandPassFilter::~BandPassFilter();
            void filter();
    };

    class KalmanFilter{
        public:
            KalmanFilter::KalmanFilter();
            KalmanFilter::~KalmanFilter();
            void start();
            double updateEstimation();
            double getKalmanGain();
            double getEstimationError();
            
        private:
            double error_measured_ {0.0};
            double error_estimated_ {0.0};
            double q_ {0.0};
            double current_estimation_ {0.0};
            double previous_estimation_ {0.0};
            double kalman_gain_ {0.0};

    };

} // sensor_filters_ns


#endif