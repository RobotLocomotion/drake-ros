#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py {
namespace {

using std_msgs::msg::Int32;

class CppPubAndSub {
 public:
  explicit CppPubAndSub(rclcpp::Node::SharedPtr node) : node_(node) {
    sub_ = node_->create_subscription<Int32>(
        "/cpp_sub", 1, [this](const Int32& message) { value_ = message.data; });
    pub_ = node_->create_publisher<Int32>("/cpp_pub", 1);
  }

  void Publish(int value) {
    Int32 message{};
    message.data = value;
    pub_->publish(std::move(message));
  }

  int SpinAndReturnLatest(double timeout_sec) {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    const std::chrono::nanoseconds timeout(static_cast<int>(timeout_sec * 1e9));
    exec.spin_some(timeout);
    return value_;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Int32>::SharedPtr sub_;
  rclcpp::Publisher<Int32>::SharedPtr pub_;
  int value_{-1};
};

PYBIND11_MODULE(drake_ros_test_pub_and_sub_cc, m) {
  py::module_::import("drake_ros.core");

  py::class_<CppPubAndSub>(m, "CppPubAndSub")
      .def(py::init<rclcpp::Node::SharedPtr>(), py::arg("node"))
      .def("Publish", &CppPubAndSub::Publish, py::arg("value"))
      .def("SpinAndReturnLatest", &CppPubAndSub::SpinAndReturnLatest,
           py::arg("timeout_sec"));
}

}  // namespace
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
