#include "drake_ros/core/rgbd_system.h"

#include <cstddef>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <drake/systems/sensors/image.h>

using drake::systems::sensors::Image;
using drake::systems::sensors::ImageRgba8U;
using drake_ros::core::RGBDSystem;
using drake_ros::core::RosPublisherSystem;

using drake::Value;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::PublishEvent;
using drake::systems::TriggerType;

RGBDSystem::RGBDSystem() {
  DeclareAbstractOutputPort("rgbd_color", &RGBDSystem::CalcColorImage);
  DeclareAbstractOutputPort("rgbd_depth", &RGBDSystem::CalcDepthImage);
}

RGBDSystem::~RGBDSystem() {}

void RGBDSystem::CalcColorImage(const drake::systems::Context<double>& context,
                                sensor_msgs::msg::Image* color_value) const {
  color_value->header.frame_id = "CartPole/camera_optical";

  rclcpp::Time now{0, 0, RCL_ROS_TIME};
  now += rclcpp::Duration::from_seconds(context.get_time());
  color_value->header.stamp = now;

  color_value->height = image_msgs.height;
  color_value->width = image_msgs.width;
  color_value->step = image_msgs.step;
  color_value->encoding = image_msgs.encoding;
  color_value->data.resize(image_msgs.data.size());
  memcpy(&color_value->data[0], &image_msgs.data[0], image_msgs.data.size());
}

void RGBDSystem::CalcDepthImage(const drake::systems::Context<double>& context,
                                sensor_msgs::msg::Image* depth_value) const {
  depth_value->header.frame_id = "CartPole/camera_optical";

  rclcpp::Time now{0, 0, RCL_ROS_TIME};
  now += rclcpp::Duration::from_seconds(context.get_time());
  depth_value->header.stamp = now;

  depth_value->height = depth_msgs.height;
  depth_value->width = depth_msgs.width;
  depth_value->step = depth_msgs.step;
  depth_value->encoding = depth_msgs.encoding;
  depth_value->data.resize(depth_msgs.data.size());

  float* depth_row = reinterpret_cast<float*>(&depth_msgs.data[0]);
  float* depth_row_value = reinterpret_cast<float*>(&depth_value->data[0]);

  for (size_t x = 0; x < depth_value->data.size() / 4; ++x) {
    depth_row_value[x] = depth_row[x];
    if (std::numeric_limits<float>::infinity() == depth_row[x]) {
      depth_row_value[x] = 0.0f;
    }
  }
}

template <PixelType kPixelType>
const InputPort<double>& RGBDSystem::DeclareDepthInputPort(
    std::string port_name, double publish_period, double start_time) {
  // Test to confirm valid pixel type.
  static_assert(kPixelType == PixelType::kRgba8U ||
                    kPixelType == PixelType::kDepth32F ||
                    kPixelType == PixelType::kDepth16U ||
                    kPixelType == PixelType::kLabel16I ||
                    kPixelType == PixelType::kGrey8U,
                "ImageWriter: the only supported pixel types are: kRgba8U, "
                "kDepth32F, kDepth16U, kGrey8U, and kLabel16I");

  if (publish_period <= 0) {
    throw std::logic_error("ImageWriter: publish period must be positive");
  }

  // Now configure the system for the valid port declaration.
  const auto& port =
      DeclareAbstractInputPort(port_name, Value<Image<kPixelType>>());
  // There is no DeclarePeriodicPublishEvent that accepts a lambda, so we must
  // use the advanced API to add our event.
  PublishEvent<double> event(
      TriggerType::kPeriodic,
      [port_index = port.get_index()](const System<double>& system,
                                      const Context<double>& context,
                                      const PublishEvent<double>&) {
        const auto& self = dynamic_cast<const RGBDSystem&>(system);
        self.PubDepth<kPixelType>(context, port_index);
        return EventStatus::Succeeded();
      });
  DeclarePeriodicEvent<PublishEvent<double>>(publish_period, start_time, event);
  port_info_.emplace_back("depth", kPixelType);
  return port;
}

template <PixelType kPixelType>
const InputPort<double>& RGBDSystem::DeclareImageInputPort(
    std::string port_name, double publish_period, double start_time) {
  // Test to confirm valid pixel type.
  static_assert(kPixelType == PixelType::kRgba8U ||
                    kPixelType == PixelType::kDepth32F ||
                    kPixelType == PixelType::kDepth16U ||
                    kPixelType == PixelType::kLabel16I ||
                    kPixelType == PixelType::kGrey8U,
                "ImageWriter: the only supported pixel types are: kRgba8U, "
                "kDepth32F, kDepth16U, kGrey8U, and kLabel16I");

  if (publish_period <= 0) {
    throw std::logic_error("ImageWriter: publish period must be positive");
  }

  // Now configure the system for the valid port declaration.
  const auto& port =
      DeclareAbstractInputPort(port_name, Value<Image<kPixelType>>());
  // There is no DeclarePeriodicPublishEvent that accepts a lambda, so we must
  // use the advanced API to add our event.
  PublishEvent<double> event(
      TriggerType::kPeriodic,
      [port_index = port.get_index()](const System<double>& system,
                                      const Context<double>& context,
                                      const PublishEvent<double>&) {
        const auto& self = dynamic_cast<const RGBDSystem&>(system);
        self.PubImage<kPixelType>(context, port_index);
        return EventStatus::Succeeded();
      });
  DeclarePeriodicEvent<PublishEvent<double>>(publish_period, start_time, event);
  port_info_.emplace_back("color", kPixelType);
  return port;
}

template <PixelType kPixelType>
void RGBDSystem::PubImage(const Context<double>& context, int index) const {
  const auto& port = get_input_port(index);
  const Image<kPixelType>& image = port.Eval<Image<kPixelType>>(context);

  image_msgs.data.resize(image.width() * image.height() * 3);
  image_msgs.height = image.height();
  image_msgs.width = image.width();
  image_msgs.step = image.width() * 3;
  image_msgs.encoding = "rgb8";
  for (int x = 0; x < image.width(); ++x) {
    for (int y = 0; y < image.height(); ++y) {
      image_msgs.data[3 * (image.width() * y + x)] = image.at(x, y)[0];
      image_msgs.data[3 * (image.width() * y + x) + 1] = image.at(x, y)[1];
      image_msgs.data[3 * (image.width() * y + x) + 2] = image.at(x, y)[2];
    }
  }
}

template <PixelType kPixelType>
void RGBDSystem::PubDepth(const Context<double>& context, int index) const {
  const auto& port = get_input_port(index);
  const Image<kPixelType>& image = port.Eval<Image<kPixelType>>(context);

  depth_msgs.data.resize(image.width() * image.height() * sizeof(float));
  depth_msgs.height = image.height();
  depth_msgs.width = image.width();
  depth_msgs.step = image.width() * sizeof(float);
  depth_msgs.encoding = "32FC1";
  float* depth_row = reinterpret_cast<float*>(&depth_msgs.data[0]);
  for (int x = 0; x < image.width(); ++x) {
    for (int y = 0; y < image.height(); ++y) {
      float value = image.at(x, y)[0];
      depth_row[image.width() * y + x] = value;
    }
  }
}

template const InputPort<double>&
RGBDSystem::DeclareImageInputPort<PixelType::kRgba8U>(std::string port_name,
                                                      double publish_period,
                                                      double start_time);
template const InputPort<double>&
RGBDSystem::DeclareImageInputPort<PixelType::kDepth32F>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareImageInputPort<PixelType::kLabel16I>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareImageInputPort<PixelType::kDepth16U>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareImageInputPort<PixelType::kGrey8U>(std::string port_name,
                                                      double publish_period,
                                                      double start_time);

template void RGBDSystem::PubImage<PixelType::kRgba8U>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubImage<PixelType::kDepth32F>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubImage<PixelType::kLabel16I>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubImage<PixelType::kDepth16U>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubImage<PixelType::kGrey8U>(
    const Context<double>& context, int index) const;

template const InputPort<double>&
RGBDSystem::DeclareDepthInputPort<PixelType::kRgba8U>(std::string port_name,
                                                      double publish_period,
                                                      double start_time);
template const InputPort<double>&
RGBDSystem::DeclareDepthInputPort<PixelType::kDepth32F>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareDepthInputPort<PixelType::kLabel16I>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareDepthInputPort<PixelType::kDepth16U>(std::string port_name,
                                                        double publish_period,
                                                        double start_time);
template const InputPort<double>&
RGBDSystem::DeclareDepthInputPort<PixelType::kGrey8U>(std::string port_name,
                                                      double publish_period,
                                                      double start_time);

template void RGBDSystem::PubDepth<PixelType::kRgba8U>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubDepth<PixelType::kDepth32F>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubDepth<PixelType::kLabel16I>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubDepth<PixelType::kDepth16U>(
    const Context<double>& context, int index) const;
template void RGBDSystem::PubDepth<PixelType::kGrey8U>(
    const Context<double>& context, int index) const;

std::pair<RosPublisherSystem*, RosPublisherSystem*> RGBDSystem::AddToBuilder(
    drake::systems::DiagramBuilder<double>* builder, DrakeRos* ros,
    const std::string& topic_name, const rclcpp::QoS& qos,
    const std::unordered_set<drake::systems::TriggerType>& publish_triggers,
    double publish_period) {
  // std::cout << "Add To builder" << std::endl;
  auto* pub_color_system =
      builder->AddSystem(RosPublisherSystem::Make<sensor_msgs::msg::Image>(
          topic_name, qos, ros, publish_triggers, publish_period));

  auto* pub_depth_system =
      builder->AddSystem(RosPublisherSystem::Make<sensor_msgs::msg::Image>(
          "depth", qos, ros, publish_triggers, publish_period));

  return {pub_color_system, pub_depth_system};
}
