#pragma once

#include "ros_idl_pybind.h"

// Generic typecaster for specific ROS 2 messages.
// This method can be used instead of the ROS_MSG_PYBIND_TYPECAST_ALL() macro.
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Quaternion);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Point);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Vector3);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Twist);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Accel);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Wrench);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Pose);
ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Transform);
