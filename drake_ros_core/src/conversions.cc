// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "drake_ros_core/conversions.h"

namespace drake_ros_core {
namespace conversions {

Eigen::Vector3d ros_point_to_eigen_vector3d(
    const geometry_msgs::msg::Point& point)
{
  Eigen::Vector3d result;
  result[0] = point.x;
  result[1] = point.y;
  result[2] = point.z;

  return result;
}

geometry_msgs::msg::Point eigen_vector3d_to_ros_point(
    const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point result;
  result.x = point[0];
  result.y = point[1];
  result.z = point[2];
  return result;
}

Eigen::Quaternion<double> ros_quat_to_eigen_quat(
    const geometry_msgs::msg::Quaternion& quat)
{
  return Eigen::Quaternion<double>(quat.w, quat.x, quat.y, quat.z);
}

geometry_msgs::msg::Quaternion eigen_quat_to_ros_quat(
    const Eigen::Quaternion<double>& quat)
{
  geometry_msgs::msg::Quaternion result;
  result.x = quat.x();
  result.y = quat.y();
  result.z = quat.z();
  result.w = quat.w();

  return result;
}

Eigen::Isometry3d ros_pose_to_eigen_isometry3d(
    const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d result;
  result.translate(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
  Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                          pose.orientation.y, pose.orientation.z);
  result.rotate(quat);
  return result;
}

geometry_msgs::msg::Pose eigen_isometry3d_to_ros_pose(
    const Eigen::Isometry3d& isometry)
{
  geometry_msgs::msg::Pose result;
  result.position.x = isometry.translation()[0];
  result.position.y = isometry.translation()[1];
  result.position.z = isometry.translation()[2];
  Eigen::Quaterniond orientation(isometry.rotation());
  result.orientation.x = orientation.x();
  result.orientation.y = orientation.y();
  result.orientation.z = orientation.z();
  result.orientation.w = orientation.w();
  return result;
}

drake::math::RigidTransformd ros_pose_to_drake_transform(
    const geometry_msgs::msg::Pose& pose)
{
  drake::math::RigidTransformd result;
  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                 pose.orientation.y, pose.orientation.z);
  result.set_rotation(orientation);
  Eigen::Vector3d translation(pose.position.x, pose.position.y,
                              pose.position.z);
  result.set_translation(translation);
  return result;
}

geometry_msgs::msg::Pose drake_transform_to_ros_pose(
    const drake::math::RigidTransformd& transform)
{
  geometry_msgs::msg::Pose result;
  result.position.x = transform.translation()[0];
  result.position.y = transform.translation()[1];
  result.position.z = transform.translation()[2];
  Eigen::Quaterniond orientation = transform.rotation().ToQuaternion();
  result.orientation.x = orientation.x();
  result.orientation.y = orientation.y();
  result.orientation.z = orientation.z();
  result.orientation.w = orientation.w();
  return result;
}

Eigen::Isometry3d ros_transform_to_eigen_isometry3d(
    const geometry_msgs::msg::Transform& transform)
{
  Eigen::Isometry3d result;
  result.translate(Eigen::Vector3d(transform.translation.x,
                                   transform.translation.y,
                                   transform.translation.z));
  Eigen::Quaterniond quat(transform.rotation.w, transform.rotation.x,
                          transform.rotation.y, transform.rotation.z);
  result.rotate(quat);
  return result;
}

geometry_msgs::msg::Transform eigen_isometry3d_to_ros_transform(
    const Eigen::Isometry3d& isometry)
{
  geometry_msgs::msg::Transform result;
  result.translation.x = isometry.translation()[0];
  result.translation.y = isometry.translation()[1];
  result.translation.z = isometry.translation()[2];
  Eigen::Quaterniond orientation(isometry.rotation());
  result.rotation.x = orientation.x();
  result.rotation.y = orientation.y();
  result.rotation.z = orientation.z();
  result.rotation.w = orientation.w();
  return result;
}

drake::math::RigidTransformd ros_transform_to_drake_transform(
    const geometry_msgs::msg::Transform& transform)
{
  drake::math::RigidTransformd result;
  Eigen::Quaterniond orientation(transform.rotation.w, transform.rotation.x,
                                 transform.rotation.y, transform.rotation.z);
  result.set_rotation(orientation);
  Eigen::Vector3d translation(transform.translation.x, transform.translation.y,
                              transform.translation.z);
  result.set_translation(translation);
  return result;
}

geometry_msgs::msg::Transform drake_transform_to_ros_transform(
    const drake::math::RigidTransformd& transform)
{
  geometry_msgs::msg::Transform result;
  result.translation.x = transform.translation()[0];
  result.translation.y = transform.translation()[1];
  result.translation.z = transform.translation()[2];
  Eigen::Quaterniond orientation = transform.rotation().ToQuaternion();
  result.rotation.x = orientation.x();
  result.rotation.y = orientation.y();
  result.rotation.z = orientation.z();
  result.rotation.w = orientation.w();
  return result;
}

drake::Vector6d ros_twist_to_eigen_vector6d(
    const geometry_msgs::msg::Twist& twist)
{
  drake::Vector6d result;
  result[0] = twist.linear.x;
  result[1] = twist.linear.y;
  result[2] = twist.linear.z;
  result[3] = twist.angular.x;
  result[4] = twist.angular.y;
  result[5] = twist.angular.z;
  return result;
}

geometry_msgs::msg::Twist eigen_vector6d_to_ros_twist(
    const drake::Vector6d& vector)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = vector[0];
  result.linear.y = vector[1];
  result.linear.z = vector[2];
  result.angular.x = vector[3];
  result.angular.y = vector[4];
  result.angular.z = vector[5];
  return result;
}

drake::multibody::SpatialVelocity<double> ros_twist_to_drake_velocity(
    const geometry_msgs::msg::Twist& twist)
{
  drake::multibody::SpatialVelocity<double> result(
      Eigen::Vector3d(twist.angular.x, twist.angular.y, twist.angular.z),
      Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.linear.z));
  return result;
}

geometry_msgs::msg::Twist drake_velocity_to_ros_twist(
    const drake::multibody::SpatialVelocity<double>& velocity)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = velocity.translational()[0];
  result.linear.y = velocity.translational()[1];
  result.linear.z = velocity.translational()[2];
  result.angular.x = velocity.rotational()[0];
  result.angular.y = velocity.rotational()[1];
  result.angular.z = velocity.rotational()[2];
  return result;
}

drake::Vector6d ros_accel_to_eigen_vector6d(
    const geometry_msgs::msg::Accel& accel)
{
  drake::Vector6d result;
  result[0] = accel.linear.x;
  result[1] = accel.linear.y;
  result[2] = accel.linear.z;
  result[3] = accel.angular.x;
  result[4] = accel.angular.y;
  result[5] = accel.angular.z;
  return result;
}

geometry_msgs::msg::Accel eigen_vector6d_to_ros_accel(
    const drake::Vector6d& vector)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = vector[0];
  result.linear.y = vector[1];
  result.linear.z = vector[2];
  result.angular.x = vector[3];
  result.angular.y = vector[4];
  result.angular.z = vector[5];
  return result;
}

drake::multibody::SpatialAcceleration<double> ros_accel_to_drake_accel(
    const geometry_msgs::msg::Accel& accel)
{
  drake::multibody::SpatialAcceleration<double> result(
      Eigen::Vector3d(accel.angular.x, accel.angular.y, accel.angular.z),
      Eigen::Vector3d(accel.linear.x, accel.linear.y, accel.linear.z));
  return result;
}

geometry_msgs::msg::Accel drake_accel_to_ros_accel(
    const drake::multibody::SpatialAcceleration<double>& accel)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = accel.translational()[0];
  result.linear.y = accel.translational()[1];
  result.linear.z = accel.translational()[2];
  result.angular.x = accel.rotational()[0];
  result.angular.y = accel.rotational()[1];
  result.angular.z = accel.rotational()[2];
  return result;
}

drake::Vector6d ros_wrench_to_eigen_vector6d(
    const geometry_msgs::msg::Wrench& wrench)
{
  drake::Vector6d result;
  result[0] = wrench.force.x;
  result[1] = wrench.force.y;
  result[2] = wrench.force.z;
  result[3] = wrench.torque.x;
  result[4] = wrench.torque.y;
  result[5] = wrench.torque.z;
  return result;
}

geometry_msgs::msg::Wrench eigen_vector6d_to_ros_wrench(
    const drake::Vector6d& vector)
{
  geometry_msgs::msg::Wrench result;
  result.force.x = vector[0];
  result.force.y = vector[1];
  result.force.z = vector[2];
  result.torque.x = vector[3];
  result.torque.y = vector[5];
  result.torque.z = vector[5];
  return result;
}

drake::multibody::SpatialForce<double> ros_wrench_to_drake_force(
    const geometry_msgs::msg::Wrench& wrench)
{
  drake::multibody::SpatialForce<double> result(
      Eigen::Vector3d(wrench.torque.x, wrench.torque.y, wrench.torque.z),
      Eigen::Vector3d(wrench.force.x, wrench.force.y, wrench.force.z));
  return result;
}

geometry_msgs::msg::Wrench drake_force_to_ros_wrench(
    const drake::multibody::SpatialForce<double>& force)
{
  geometry_msgs::msg::Wrench result;
  result.force.x = force.translational()[0];
  result.force.y = force.translational()[1];
  result.force.z = force.translational()[2];
  result.torque.x = force.rotational()[0];
  result.torque.y = force.rotational()[1];
  result.torque.z = force.rotational()[2];
  return result;
}

}  // namespace conversions
}  // namespace drake_ros_core
