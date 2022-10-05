#ifndef SIGNAL_FILTER_
#define SIGNAL_FILTER_

#include <algorithm>

namespace signal_filter {
template <typename T>
T thresholdFilter(const T &min, const T &max, const T &input)
{
    return std::clamp(input, min, max);
}

template <typename T, typename T2>
T thresholdFilter(const T2 &min, const T2 &max, const T &input)
{
    return std::clamp(input, static_cast<T>(min), static_cast<T>(max));
}

template <typename T>
T lowPassFilter(const double &gain, const T &curr_input, const T &prev_input)
{
    return curr_input * (gain) + prev_input * (1 - gain);
}
} // namespace signal_filter

#endif