#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/systems/sensors/rgbd_sensor.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

using drake::systems::Context;
using drake::systems::InputPort;
using drake::systems::sensors::PixelType;
using drake::systems::sensors::RgbdSensor;

namespace drake_ros::core {

/** A system that convert's drake's time to a sensor_msgs/msg/Image message.
 */
class RGBDSystem : public drake::systems::LeafSystem<double> {
 public:
  /** A constructor for the rbgd system.
   */
  RGBDSystem();

  ~RGBDSystem() override;

  template <PixelType kPixelType>
  const InputPort<double>& DeclareImageInputPort(std::string port_name,
                                                 double publish_period,
                                                 double start_time);

  template <PixelType kPixelType>
  const InputPort<double>& DeclareDepthInputPort(std::string port_name,
                                                 double publish_period,
                                                 double start_time);

  // The per-input port data.
  struct RgbdSensorPortInfo {
    RgbdSensorPortInfo(std::string format_in, PixelType pixel_type_in)
        : format(std::move(format_in)), pixel_type(pixel_type_in) {}
    const std::string format;
    const PixelType pixel_type;
    // NOTE: This is made mutable as a low-cost mechanism for incrementing
    // image writes without involving the overhead of discrete state.
    mutable int count{0};
    // TODO(SeanCurtis-TRI): For copying this system, it may be necessary to
    // also store the period and start time so that the ports in the copy can
    // be properly instantiated.
  };

  template <PixelType kPixelType>
  void PubImage(const Context<double>&, int) const;

  template <PixelType kPixelType>
  void PubDepth(const Context<double>&, int) const;

  // For each input port, this stores the corresponding image data. It is an
  // invariant that port_info_.size() == num_input_ports().
  std::vector<RgbdSensorPortInfo> port_info_;

  // void SetSystem(RgbdSensor * rgbd_system);

  /** Add a RGBDSystem and RosPublisherSystem to a diagram builder.
   *
   * This adds both a RGBDSystem and a RosPublisherSystem that publishes
   * time to a `/image` topic. All nodes should have their `use_sim_time`
   * parameter set to `True` so they use the published topic as their source of
   * time.
   */
  static std::pair<RosPublisherSystem*, RosPublisherSystem*> AddToBuilder(
      drake::systems::DiagramBuilder<double>* builder, DrakeRos* ros,
      const std::string& topic_name = "/image",
      const rclcpp::QoS& qos = rclcpp::SystemDefaultsQoS(),
      const std::unordered_set<drake::systems::TriggerType>& publish_triggers =
          RosPublisherSystem::kDefaultTriggerTypes,
      double publish_period = 0.0);

 private:
  mutable sensor_msgs::msg::Image image_msgs;
  mutable sensor_msgs::msg::Image depth_msgs;

 protected:
  void CalcColorImage(const drake::systems::Context<double>& context,
                      sensor_msgs::msg::Image* color_value) const;

  void CalcDepthImage(const drake::systems::Context<double>& context,
                      sensor_msgs::msg::Image* depth_value) const;
};
}  // namespace drake_ros::core
