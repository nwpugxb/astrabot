#pragma once

#include <rmw/qos_profiles.h>

// micro-ROS rclc default depth is 10; use 1 for low-latency sensor + cmd topics.
static inline rmw_qos_profile_t microros_qos_depth1(void) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  return qos;
}
