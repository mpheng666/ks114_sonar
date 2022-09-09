#ifndef _SENSOR_FILTERS_HPP_
#define _SENSOR_FILTERS_HPP_

namespace sensor_filters_ns{

    template<typename T>
    class ThresholdFilter{
        public:
            ThresholdFilter(const T& min, const T& max, const T& return_value)
            :
                min_limit_(min),
                max_limit_(max),
                return_value_(return_value)
            {
            }

            ~ThresholdFilter()
            {

            }

            const T filter(const T& input)
            {
                return (input > min_limit_ && input < max_limit_) ? input : return_value_;
            }

            const T& getMinLimit() const 
            {
                return min_limit_;
            }

            const T& getMaxLimit() const 
            {
                return max_limit_;
            }

            bool setMinLimit(const T& min)
            {
                min_limit_ = min;
                return true;
            }

            bool setMaxLimit(const T& max)
            {
                max_limit_ = max;
                return true;
            }

        private:
            T min_limit_;
            T max_limit_;
            T return_value_;
    };

    template<typename T>
    class LowPassFilter{
        public:
            LowPassFilter(const double& gain)
            :
                low_pass_gain_(gain)
            {

            }

            ~LowPassFilter()
            {

            }

            const T filter(const T& input, const T& input_previous)
            {
                return low_pass_gain_ * input + (1 - low_pass_gain_) * input_previous;
            }

            bool setGain(const double& gain)
            {
                low_pass_gain_ = gain;
            }

            const T& getGain() const
            {
                return low_pass_gain_;
            }

        private:
            double low_pass_gain_ {0.0};
    };

    class MedianFilter{
        public:
            MedianFilter();
            ~MedianFilter();
            void filter();
    };

    class BandPassFilter{
        public:
            BandPassFilter();
            ~BandPassFilter();
            void filter();
    };

    class KalmanFilter{
        public:
            KalmanFilter();
            ~KalmanFilter();
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