/** @file
 An example of visualizing ContactSurfaces with RViz.  */

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "drake/multibody/meshcat/contact_visualizer.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <drake/geometry/kinematics_vector.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/simulator_config.h>
#include <drake/systems/analysis/simulator_config_functions.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/visualization/visualization_config.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/viz/contact_markers_system.h>
#include <drake_ros/viz/rviz_visualizer.h>
#include <gflags/gflags.h>
#include <visualization_msgs/msg/marker_array.hpp>

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizerd;
using drake::geometry::MeshcatVisualizerParams;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ContactModel;
using drake::multibody::Parser;
using drake::multibody::meshcat::ContactVisualizerd;
using drake::multibody::meshcat::ContactVisualizerParams;
using drake::systems::ApplySimulatorConfig;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::SimulatorConfig;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;
using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::viz::ConnectContactResultsToRviz;
using drake_ros::viz::ContactMarkersParams;
using drake_ros::viz::RvizVisualizer;

using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "How many seconds to run the simulation");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_bool(use_meshcat, false, "Enable meshcat visualizer.");

namespace drake_ros_examples {
void AddScene(MultibodyPlantd* plant) {
  auto parser = Parser(plant);
  const auto package_path =
      ament_index_cpp::get_package_share_directory("drake_ros_examples");
  parser.package_map().Add("drake_ros_examples", package_path);

  std::filesystem::path fs_path{
      parser.package_map().GetPath("drake_ros_examples")};
  parser.AddModels((fs_path / "hydroelastic/hydroelastic.sdf").string());
}

int do_main() {
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  AddScene(&plant);

  plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
  plant.Finalize();

  std::shared_ptr<Meshcat> meshcat;
  if (FLAGS_use_meshcat) {
    meshcat = std::make_shared<Meshcat>();
    const VisualizationConfig config;
    ApplyVisualizationConfig(config, &builder, nullptr, &plant, &scene_graph,
                             meshcat);

    ContactVisualizerParams cparams;
    ContactVisualizerd::AddToBuilder(&builder, plant, meshcat,
                                     std::move(cparams));
  }

  // Visualize with RViz
  drake_ros::core::init();
  auto ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("collisions"));

  auto& rviz_visualizer = *builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());

  rviz_visualizer.RegisterMultibodyPlant(&plant);

  builder.Connect(scene_graph.get_query_output_port(),
                  rviz_visualizer.get_graph_query_input_port());

  ConnectContactResultsToRviz(&builder, plant, scene_graph,
                              ros_interface_system->get_ros_interface());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  Simulator<double> simulator(*diagram, std::move(diagram_context));
  SimulatorConfig conf;
  conf.target_realtime_rate = FLAGS_real_time ? 1.f : 0.f;
  ApplySimulatorConfig(conf, &simulator);

  auto& simulator_context = simulator.get_mutable_context();

  simulator.Initialize();

  // Step the simulator in 0.1s intervals
  constexpr double kStep{0.1};
  while (simulator_context.get_time() < FLAGS_simulation_sec) {
    const double next_time =
        std::min(FLAGS_simulation_sec, simulator_context.get_time() + kStep);
    simulator.AdvanceTo(next_time);
  }

  return 0;
}
}  // namespace drake_ros_examples

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake_ros_examples::do_main();
}
