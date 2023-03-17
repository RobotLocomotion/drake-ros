#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <drake/common/eigen_types.h>
#include <drake/common/value.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "drake_ros/tf2/scene_tf_broadcaster_system.h"

using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::tf2::SceneTfBroadcasterParams;
using drake_ros::tf2::SceneTfBroadcasterSystem;

TEST(SceneTfBroadcasting, NominalCase) {
  drake_ros::core::init();

  drake::systems::DiagramBuilder<double> builder;

  auto system_ros = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("tf_broadcaster"));

  auto scene_graph = builder.AddSystem<drake::geometry::SceneGraph>();
  const drake::geometry::SourceId source_id =
      scene_graph->RegisterSource("test_source");
  const drake::geometry::FrameId odom_frame = scene_graph->RegisterFrame(
      source_id, drake::geometry::GeometryFrame("odom"));
  const drake::geometry::FrameId base_frame = scene_graph->RegisterFrame(
      source_id, odom_frame, drake::geometry::GeometryFrame("base_link"));

  const drake::math::RigidTransform<double> X_WO{
      // World to Odom
      drake::math::RotationMatrix<double>::Identity(),
      drake::Vector3<double>{1., 1., 0.}};
  const drake::math::RigidTransform<double> X_OB{
      // Odom to Base link
      drake::math::RollPitchYaw<double>(0., 0., M_PI / 2.),
      drake::Vector3<double>{0., 0., 0.1}};

  const drake::geometry::FramePoseVector<double> pose_vector{
      {odom_frame, X_WO}, {base_frame, X_OB}};

  auto pose_vector_value = drake::AbstractValue::Make(pose_vector);
  auto pose_vector_source =
      builder.AddSystem<drake::systems::ConstantValueSource>(
          *pose_vector_value);

  builder.Connect(pose_vector_source->get_output_port(),
                  scene_graph->get_source_pose_port(source_id));

  auto scene_tf_broadcaster = builder.AddSystem<SceneTfBroadcasterSystem>(
      system_ros->get_ros_interface(),
      SceneTfBroadcasterParams{
          {drake::systems::TriggerType::kForced}, 0., "/tf"});

  builder.Connect(scene_graph->get_query_output_port(),
                  scene_tf_broadcaster->GetInputPort("graph_query"));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Don't need to rclcpp::init because DrakeRos uses global rclcpp::Context by
  // default
  auto node = rclcpp::Node::make_shared("tf_listener");

  tf2_ros::Buffer buffer(node->get_clock());
  buffer.setUsingDedicatedThread(true);  // Shut tf2 warnings.

  constexpr bool kNoSpinThread{false};
  tf2_ros::TransformListener listener(buffer, node, kNoSpinThread);

  const auto time = rclcpp::Time() + rclcpp::Duration::from_seconds(13.);

  context->SetTime(time.seconds());
  diagram->ForcedPublish(*context);
  rclcpp::spin_some(node);

  EXPECT_TRUE(buffer.canTransform("world", "odom", time));
  const geometry_msgs::msg::TransformStamped world_to_odom =
      buffer.lookupTransform("world", "odom", time);
  const builtin_interfaces::msg::Time stamp = time;
  EXPECT_EQ(world_to_odom.header.stamp.sec, stamp.sec);
  EXPECT_EQ(world_to_odom.header.stamp.nanosec, stamp.nanosec);
  const drake::Vector3<double>& p_WO = X_WO.translation();
  EXPECT_DOUBLE_EQ(world_to_odom.transform.translation.x, p_WO.x());
  EXPECT_DOUBLE_EQ(world_to_odom.transform.translation.y, p_WO.y());
  EXPECT_DOUBLE_EQ(world_to_odom.transform.translation.z, p_WO.z());
  const Eigen::Quaternion<double> R_WO = X_WO.rotation().ToQuaternion();
  EXPECT_DOUBLE_EQ(world_to_odom.transform.rotation.x, R_WO.x());
  EXPECT_DOUBLE_EQ(world_to_odom.transform.rotation.y, R_WO.y());
  EXPECT_DOUBLE_EQ(world_to_odom.transform.rotation.z, R_WO.z());
  EXPECT_DOUBLE_EQ(world_to_odom.transform.rotation.w, R_WO.w());

  EXPECT_TRUE(buffer.canTransform("odom", "base_link", time));
  const geometry_msgs::msg::TransformStamped odom_to_base_link =
      buffer.lookupTransform("odom", "base_link", time);
  EXPECT_EQ(odom_to_base_link.header.stamp.sec, stamp.sec);
  EXPECT_EQ(odom_to_base_link.header.stamp.nanosec, stamp.nanosec);
  const drake::Vector3<double>& p_OB = X_OB.translation();
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.translation.x, p_OB.x());
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.translation.y, p_OB.y());
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.translation.z, p_OB.z());
  const Eigen::Quaternion<double> R_OB = X_OB.rotation().ToQuaternion();
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.rotation.x, R_OB.x());
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.rotation.y, R_OB.y());
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.rotation.z, R_OB.z());
  EXPECT_DOUBLE_EQ(odom_to_base_link.transform.rotation.w, R_OB.w());

  EXPECT_TRUE(drake_ros::core::shutdown());
}

// Only available in Bazel.
#ifndef _TEST_DISABLE_RMW_ISOLATION
#include "rmw_isolation/rmw_isolation.h"

int main(int argc, char* argv[]) {
  const char* TEST_TMPDIR = std::getenv("TEST_TMPDIR");
  if (TEST_TMPDIR != nullptr) {
    std::string ros_home = std::string(TEST_TMPDIR) + "/.ros";
    setenv("ROS_HOME", ros_home.c_str(), 1);
    ros2::isolate_rmw_by_path(argv[0], TEST_TMPDIR);
  }

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#endif
