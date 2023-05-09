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

  explicit QoS(const rclcpp::QoS& other) : rclcpp::QoS(other) {}
};
}  // namespace drake_ros

namespace drake_ros {
namespace drake_ros_py {
using std::enable_if_t;
using std::remove_cv_t;
using handle = py::handle;
using object = py::object;
using return_value_policy = py::return_value_policy;
using none = py::none;
// TODO(hidmic): rely on rclpy.qos.QoSProfile when and if a pybind11 binding
// is introduced upstream. See https://github.com/ros2/rclpy/issues/843.
template <typename T>
struct qos_type_caster {
 public:
  // declares local 'value' of type QoS
  PYBIND11_TYPE_CASTER(drake_ros::QoS, py::detail::_("rclpy.qos.QoSProfile"));

  // Convert from Python rclpy.qos.QoSProfile to QoS
  bool load(handle src, bool) {
    handle cls = py::module::import("rclpy.qos").attr("QoSProfile");
    if (!isinstance(src, cls)) {
      return false;
    }

    object source = py::reinterpret_borrow<object>(src);
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

    value.liveliness(static_cast<rmw_qos_liveliness_policy_t>(
        source.attr("liveliness").cast<ssize_t>()));

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
    (void)policy;
    (void)parent;

    const auto& rmw_qos = src.get_rmw_qos_profile();

    object duration = py::module::import("rclpy.duration").attr("Duration");

    object lifespan_duration =
        duration(pybind11::arg("seconds") = rmw_qos.lifespan.sec,
                 pybind11::arg("nanoseconds") = rmw_qos.lifespan.nsec);

    object deadline_duration =
        duration(pybind11::arg("seconds") = rmw_qos.deadline.sec,
                 pybind11::arg("nanoseconds") = rmw_qos.deadline.nsec);

    object liveliness_lease_duration = duration(
        pybind11::arg("seconds") = rmw_qos.liveliness_lease_duration.sec,
        pybind11::arg("nanoseconds") = rmw_qos.liveliness_lease_duration.nsec);

    object instance =
        py::module::import("rclpy.qos")
            .attr("QoSProfile")(
                pybind11::arg("history") =
                    static_cast<ssize_t>(rmw_qos.history),
                pybind11::arg("depth") = static_cast<size_t>(rmw_qos.depth),
                pybind11::arg("reliability") =
                    static_cast<ssize_t>(rmw_qos.reliability),
                pybind11::arg("durability") =
                    static_cast<ssize_t>(rmw_qos.durability),
                pybind11::arg("lifespan") = lifespan_duration,
                pybind11::arg("deadline") = deadline_duration,
                pybind11::arg("liveliness") =
                    static_cast<ssize_t>(rmw_qos.liveliness),
                pybind11::arg("liveliness_lease_duration") =
                    liveliness_lease_duration,
                pybind11::arg("avoid_ros_namespace_conventions") =
                    rmw_qos.avoid_ros_namespace_conventions);
    instance.inc_ref();
    return instance;
  }
};
}  // namespace drake_ros_py
}  // namespace drake_ros

namespace pybind11 {
namespace detail {
template <>
struct type_caster<drake_ros::QoS>
    : public drake_ros::drake_ros_py::qos_type_caster<drake_ros::QoS> {};
}  // namespace detail
}  // namespace pybind11
