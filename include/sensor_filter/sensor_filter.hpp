#ifndef _KS114_SONAR_KALMAN_FILTER_HPP_
#define _KS114_SONAR_KALMAN_FILTER_HPP_

namespace ks114_ns{
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
} //ks114_ns


#endif