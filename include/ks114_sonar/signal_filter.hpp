#ifndef KS114_SONAR_SIGNAL_FILTER_HPP_
#define KS114_SONAR_SIGNAL_FILTER_HPP_

#include <algorithm>

namespace signal_filter {

template <typename T>
static constexpr T thresholdFilter(const T &min, const T &max, const T &input)
{
    return std::clamp(input, min, max);
}

template <typename T, typename T2>
static constexpr T thresholdFilter(const T2 &min, const T2 &max, const T &input)
{
    return std::clamp(input, static_cast<T>(min), static_cast<T>(max));
}

template <typename T>
static constexpr T
lowPassFilter(const double &gain, const T &curr_input, const T &prev_input)
{
    return curr_input * (gain) + prev_input * (1 - gain);
}
} // namespace signal_filter

#endif