#pragma once

#include <memory>

#include <pybind11/pybind11.h>
#include <rclcpp/qos.hpp>

#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
// Thin wrapper that adds a default constructor, since rclcpp::QoS deletes
// its own and PYBIND11_TYPE_CASTER requires one.
class QoS : public rclcpp::QoS {
 public:
  QoS() : rclcpp::QoS(1) {}
};
}  // namespace drake_ros

namespace pybind11 {
namespace detail {
// TODO(hidmic): rely on rclpy.qos.QoSProfile when and if a pybind11 binding
// is introduced upstream. See https://github.com/ros2/rclpy/issues/843.
template <>
struct type_caster<drake_ros::QoS> {
 public:
  // declares local 'value' of type QoS
  PYBIND11_TYPE_CASTER(drake_ros::QoS, _("rclpy.qos.QoSProfile"));

  // Convert from Python rclpy.qos.QoSProfile to QoS
  bool load(handle src, bool) {
    handle cls = module::import("rclpy.qos").attr("QoSProfile");
    if (!isinstance(src, cls)) {
      return false;
    }

    object source = reinterpret_borrow<object>(src);
    // history                          : enum int
    // depth                            : int
    // reliability                      : enum int
    // durability                       : enum int
    // lifespan                         : rclpy.Duration
    // deadline                         : rclpy.Duration
    // liveliness                       : enum int
    // liveliness_lease_duration        : rclpy.Duration
    // avoid_ros_namespace_conventions  : bool

    value.history(static_cast<rmw_qos_history_policy_t>(
        source.attr("history").cast<ssize_t>()));

    if (value.history() == rclcpp::HistoryPolicy::KeepLast) {
      value.keep_last(source.attr("depth").cast<size_t>());
    }

    value.reliability(static_cast<rmw_qos_reliability_policy_t>(
        source.attr("reliability").cast<ssize_t>()));

    value.durability(static_cast<rmw_qos_durability_policy_t>(
        source.attr("durability").cast<ssize_t>()));

    value.lifespan(rclcpp::Duration::from_nanoseconds(
        source.attr("lifespan").attr("nanoseconds").cast<int64_t>()));

    value.deadline(rclcpp::Duration::from_nanoseconds(
        source.attr("deadline").attr("nanoseconds").cast<int64_t>()));

    value.liveliness_lease_duration(rclcpp::Duration::from_nanoseconds(
        source.attr("liveliness_lease_duration")
            .attr("nanoseconds")
            .cast<int64_t>()));

    value.avoid_ros_namespace_conventions(
        source.attr("avoid_ros_namespace_conventions").cast<bool>());

    return true;
  }

  // Convert from Python QoS to rclpy.qos.QoSProfile
  static handle cast(drake_ros::QoS src, return_value_policy policy,
                     handle parent) {
    (void)src;
    (void)policy;
    (void)parent;
    Py_RETURN_NOTIMPLEMENTED;
  }
};
}  // namespace detail
}  // namespace pybind11
