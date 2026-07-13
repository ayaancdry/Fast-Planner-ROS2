#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <cassert>
#include <cstdio>
#include <sstream>

#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

// roscpp's ROS_ASSERT_MSG has no rclcpp equivalent; this keeps the same
// "print message, then assert" behavior without pulling in an rclcpp dependency.
#define UAV_UTILS_ASSERT_MSG(cond, ...)     \
  do {                                      \
    if (!(cond)) {                          \
      std::fprintf(stderr, __VA_ARGS__);    \
      std::fprintf(stderr, "\n");           \
    }                                       \
    assert(cond);                           \
  } while (0)

namespace uav_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  UAV_UTILS_ASSERT_MSG(low < high, "%f < %f?", (double)low, (double)high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  UAV_UTILS_ASSERT_MSG(limit > 0, "%f > 0?", (double)limit);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  UAV_UTILS_ASSERT_MSG(low < high, "%f < %f?", (double)low, (double)high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  UAV_UTILS_ASSERT_MSG(limit > 0, "%f > 0?", (double)limit);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif
