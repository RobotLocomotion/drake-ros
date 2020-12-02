#include <drake/systems/framework/leaf_system.h>

#include <pybind11/pybind11.h>

#include <drake_ros_systems/ros_publisher_system.hpp>

#include "py_ros_interface_system.hpp"
#include "py_serializer.hpp"

namespace py = pybind11;

using drake::systems::LeafSystem;

using drake_ros_systems::DrakeRos;
using drake_ros_systems::DrakeRosInterface;
using drake_ros_systems::PySerializer;
using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RosPublisherSystem;
using drake_ros_systems::SerializerInterface;


PYBIND11_MODULE(drake_ros_systems, m) {
  m.doc() = "Python wrapper for drake_ros_systems";

  py::module::import("pydrake.systems.framework");

  // Use std::shared_ptr holder so pybind11 doesn't try to delete interfaces returned from
  // get_ros_interface
  py::class_<DrakeRosInterface, std::shared_ptr<DrakeRosInterface>>(m, "DrakeRosInterface");

  py::class_<RosInterfaceSystem, LeafSystem<double>>(m, "RosInterfaceSystem")
    .def(py::init([](){
      return std::make_unique<RosInterfaceSystem>(std::make_unique<DrakeRos>());}))
    .def("get_ros_interface", &RosInterfaceSystem::get_ros_interface);

  py::class_<RosPublisherSystem, LeafSystem<double>>(m, "RosPublisherSystem")
    .def(py::init([](
      pybind11::object type,
      const char * topic_name,
      std::shared_ptr<DrakeRosInterface> ros_interface)
    {
      std::unique_ptr<SerializerInterface> serializer = std::make_unique<PySerializer>(type);
      return std::make_unique<RosPublisherSystem>(
        serializer,
        topic_name,
        rclcpp::QoS(10), // TODO(sloretz) Custom cast for rclpy.QoSProfile <--> rclcpp::Qos
        ros_interface);
    }));


  /*
  py::class_<rccl::ROS>(m, "ROS")
    .def(py::init())
    .def("spin", &rccl::ROS::spin);

  py::class_<rccl::PingPong>(m, "PingPong")
    .def(py::init<rccl::ROS &>())
    .def("ping", &rccl::PingPong::ping,
        py::arg("msg") = "C++ ping");
  */
}
