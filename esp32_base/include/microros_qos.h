#pragma once

#include <rmw/qos_profiles.h>
#include <rclc/rclc.h>

// /motor_ff_pwm — reliable, depth 1.
static inline rmw_qos_profile_t microros_qos_reliable_depth1(void) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  return qos;
}

// /odom, /imu/data_raw, /cmd_vel — best effort, depth 1 (drop stale under serial load).
static inline rmw_qos_profile_t microros_qos_best_effort_depth1(void) {
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  return qos;
}

static inline rcl_ret_t microros_publisher_init_best_effort_depth1(
    rcl_publisher_t *publisher, rcl_node_t *node,
    const rosidl_message_type_support_t *type_support, const char *topic_name) {
  rmw_qos_profile_t qos = microros_qos_best_effort_depth1();
  return rclc_publisher_init(publisher, node, type_support, topic_name, &qos);
}
