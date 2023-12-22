#include <memory>

#include <gflags/gflags.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "drake/common/drake_assert.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/camera_info_system.h>
#include <drake_ros/core/clock_system.h>
#include <drake_ros/core/rgbd_system.h>
#include <drake_ros/viz/rviz_visualizer.h>

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/image_writer.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include <drake_ros/tf2/scene_tf_broadcaster_system.h>

#include "drake/geometry/drake_visualizer.h"

#include "drake/geometry/render_vtk/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cart_pole {
namespace {

using geometry::SceneGraph;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;

using drake_ros::core::CameraInfoSystem;
using drake_ros::core::ClockSystem;
using drake_ros::core::RGBDSystem;
using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::viz::RvizVisualizer;


using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;

using Eigen::Vector3d;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;

using drake::lcm::DrakeLcm;

using drake::systems::sensors::PixelType;
using drake::systems::sensors::RgbdSensor;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::sensors::ImageToLcmImageArrayT;
using drake::systems::sensors::ImageWriter;
using drake::systems::TriggerType;

using drake::geometry::RenderEngineVtkParams;
using drake::multibody::ContactModel;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 100.0,
              "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

RigidTransformd ParseCameraPose(const std::string& input_str) {
  const char delimiter = ',';
  std::vector<double> xyzrpy_numeric;

  size_t pos = 0;
  size_t next = 0;
  while ((next = input_str.find(delimiter, pos)) != std::string::npos) {
    xyzrpy_numeric.push_back(std::stod(input_str.substr(pos, next)));
    pos = next + 1;
  }
  xyzrpy_numeric.push_back(std::stod(input_str.substr(pos, next)));
  DRAKE_DEMAND(xyzrpy_numeric.size() == 6);

  const RigidTransformd X_WB(
      RollPitchYawd{xyzrpy_numeric[3], xyzrpy_numeric[4], xyzrpy_numeric[5]},
      Vector3d(xyzrpy_numeric[0], xyzrpy_numeric[1], xyzrpy_numeric[2]));

  return X_WB;
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  // Make and add the cart_pole model.
  auto [cart_pole, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);

  scene_graph.AddRenderer("renderer",
                           MakeRenderEngineVtk(RenderEngineVtkParams()));

  auto parser = Parser(&cart_pole);

  const auto package_path =
      ament_index_cpp::get_package_share_directory("drake_ros_examples");
  parser.package_map().Add("drake_ros_examples", package_path);
  std::filesystem::path fs_path{
      parser.package_map().GetPath("drake_ros_examples")};

  const std::string sdf_url =
      (fs_path / "rgbd/rgbd.sdf").string();
  Parser(&cart_pole, &scene_graph).AddAllModelsFromFile(sdf_url);

  // visualization::AddDefaultVisualization(&builder);

  drake_ros::core::init();
  auto ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("cart_pole"));

  auto& rviz_visualizer = *builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());

  rviz_visualizer.RegisterMultibodyPlant(&cart_pole);

  builder.Connect(scene_graph.get_query_output_port(),
                  rviz_visualizer.get_graph_query_input_port());

  ClockSystem::AddToBuilder(&builder,
                            ros_interface_system->get_ros_interface());

  auto camera_info_system = CameraInfoSystem::AddToBuilder(&builder,
                                 ros_interface_system->get_ros_interface());

  auto depth_camera_info_system = CameraInfoSystem::AddToBuilder(&builder,
                                 ros_interface_system->get_ros_interface(),
                                 "/depth/camera_info");

  DrakeLcm lcm;
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);
  const ColorRenderCamera color_camera{
      {"renderer", {640, 480, M_PI_4}, {0.01, 10.0}, {}}, false};
  const DepthRenderCamera depth_camera{color_camera.core(), {0.01, 10.0}};
  const RigidTransformd X_WB = ParseCameraPose("0.8, 0.0, 0.7, -2.2, 0.0, 1.57");

  std::get<0>(camera_info_system)->SetCameraInfo(color_camera.core().intrinsics());
  std::get<0>(depth_camera_info_system)->SetCameraInfo(depth_camera.core().intrinsics());

  const auto world_id = scene_graph.world_frame_id();
  RgbdSensor* camera =
      builder.AddSystem<RgbdSensor>(world_id, X_WB, color_camera, depth_camera);
  builder.Connect(scene_graph.get_query_output_port(),
                  camera->query_object_input_port());

  // Broadcast images via LCM for visualization (requires #18862 to see them).
  const double image_publish_period = 1. / 30;
  ImageToLcmImageArrayT* image_to_lcm_image_array =
      builder.template AddSystem<ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");

  LcmPublisherSystem* image_array_lcm_publisher{nullptr};
  image_array_lcm_publisher =
      builder.template AddSystem(LcmPublisherSystem::Make<lcmt_image_array>(
          "DRAKE_RGBD_CAMERA_IMAGES", &lcm, image_publish_period));
  image_array_lcm_publisher->set_name("publisher");

  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

//   ImageWriter* image_writer{nullptr};
//   image_writer = builder.template AddSystem<ImageWriter>();

  RGBDSystem* rgbd_publisher{nullptr};
  rgbd_publisher = builder.template AddSystem<RGBDSystem>();

  const std::string filename =
      (std::filesystem::path("/home/ahcorde/drake_ros_ws/images") / "{image_type}_{count:03}")
          .string();

  const auto& port =
      image_to_lcm_image_array->DeclareImageInputPort<PixelType::kDepth32F>(
          "depth");
  builder.Connect(camera->depth_image_32F_output_port(), port);

  const auto& port_color =
      image_to_lcm_image_array->DeclareImageInputPort<PixelType::kRgba8U>(
          "color");
  builder.Connect(camera->color_image_output_port(), port_color);

  const auto& rgbd_port =
      rgbd_publisher->DeclareImageInputPort<PixelType::kRgba8U>(
          "color", image_publish_period, 0.);
  builder.Connect(camera->color_image_output_port(), rgbd_port);

  const auto& depth_port =
      rgbd_publisher->DeclareDepthInputPort<PixelType::kDepth32F>(
          "depth", image_publish_period, 0.);
  builder.Connect(camera->depth_image_32F_output_port(), depth_port);

  const double viz_dt = 1 / 32.0;
  // Add a TF2 broadcaster to provide task frame information
  auto scene_tf_broadcaster =
      builder.AddSystem<drake_ros::tf2::SceneTfBroadcasterSystem>(
          ros_interface_system->get_ros_interface(),
          drake_ros::tf2::SceneTfBroadcasterParams{
              {TriggerType::kPeriodic}, viz_dt, "/tf"});
  builder.Connect(scene_graph.get_query_output_port(),
                  scene_tf_broadcaster->get_graph_query_input_port());

//   const auto& writer_port =
//       image_writer->DeclareImageInputPort<PixelType::kDepth32F>(
//           "depth", filename, image_publish_period, 0.);
//   builder.Connect(camera->depth_image_32F_output_port(), writer_port);

//   const auto& writer_port_color =
//       image_writer->DeclareImageInputPort<PixelType::kRgba8U>(
//           "color", filename, image_publish_period, 0.);
//   builder.Connect(camera->color_image_output_port(), writer_port_color);

  auto [pub_color_system, pub_depth_system] =
    RGBDSystem::AddToBuilder(&builder,
                             ros_interface_system->get_ros_interface());

  builder.Connect(rgbd_publisher->GetOutputPort("rgbd_color"),
                  pub_color_system->get_input_port());

  builder.Connect(rgbd_publisher->GetOutputPort("rgbd_depth"),
                  pub_depth_system->get_input_port());

  // Now the model is complete.
//   cart_pole.set_contact_model(ContactModel::kPoint);
  cart_pole.Finalize();

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& cart_pole_context =
      diagram->GetMutableSubsystemContext(cart_pole, diagram_context.get());

  // There is no input actuation in this example for the passive dynamics.
  cart_pole.get_actuation_input_port().FixValue(&cart_pole_context, 0.);

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");

  // Set initial state.
  cart_slider.set_translation(&cart_pole_context, 0.0);
  pole_pin.set_angle(&cart_pole_context, 2.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple cart pole demo using Drake's MultibodyPlant,"
      "with SceneGraph visualization. "
      "Launch meldis before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::cart_pole::do_main();
}
