/** @file
 An example of visualizing ContactSurfaces with RViz.  */

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "drake/multibody/meshcat/contact_visualizer.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <drake/common/value.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/kinematics_vector.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/query_results/contact_surface.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/contact_results_to_lcm.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/analysis/simulator_config.h>
#include <drake/systems/analysis/simulator_config_functions.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/viz/contact_markers_system.h>
#include <drake_ros/viz/rviz_visualizer.h>
#include <gflags/gflags.h>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef BAZEL
// Bazel can't run programs that expect the filesystem hierarchy standard.
// We must know about bazel runfiles to access files when built by bazel.
#include "tools/cpp/runfiles/runfiles.h"

using bazel::tools::cpp::runfiles::Runfiles;
#endif

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::AddRigidHydroelasticProperties;
using drake::geometry::HalfSpace;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizerd;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::multibody::meshcat::ContactVisualizerd;
using drake::multibody::meshcat::ContactVisualizerParams;
using drake::systems::ApplySimulatorConfig;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::SimulatorConfig;
using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_viz::ConnectContactResultsToRviz;
using drake_ros_viz::ContactMarkersParams;
using drake_ros_viz::RvizVisualizer;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::make_unique;

using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "How many seconds to run the simulation");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");

DEFINE_bool(use_meshcat, false, "Enable meshcat visualizer.");

namespace drake_ros_examples {
void AddScene(const std::string& package_path, MultibodyPlantd* plant) {
  auto parser = Parser(plant);
  parser.package_map().Add("drake_ros_examples", package_path);

  std::filesystem::path fs_path{
      parser.package_map().GetPath("drake_ros_examples")};
  parser.AddAllModelsFromFile(
      (fs_path / "hydroelastic/hydroelastic.sdf").string());
}

int do_main(int argc, char** argv) {
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

#ifdef BAZEL
  {
    std::string error;
#ifdef BAZEL_CURRENT_REPOSITORY
    // bazel 6
    std::unique_ptr<Runfiles> runfiles(
        Runfiles::Create(argv[0], BAZEL_CURRENT_REPOSITORY, &error));
#else
    // bazel 5
    std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
#endif
    if (nullptr == runfiles) {
      throw std::runtime_error(error);
    }
    AddScene(runfiles->Rlocation("drake_ros_examples/examples/"), &plant);
  }
#else
  (void)argc;
  (void)argv;
  AddScene(ament_index_cpp::get_package_share_directory("drake_ros_examples"),
           &plant);
#endif

  plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
  plant.Finalize();

  std::shared_ptr<Meshcat> meshcat;
  if (FLAGS_use_meshcat) {
    meshcat = std::make_shared<Meshcat>();
    // Visualize with meshcat
    MeshcatVisualizerParams params;
    MeshcatVisualizerd::AddToBuilder(&builder, scene_graph, meshcat,
                                     std::move(params));

    ContactVisualizerParams cparams;
    ContactVisualizerd::AddToBuilder(&builder, plant, meshcat,
                                     std::move(cparams));
  }

  // Visualize with RViz
  drake_ros_core::init();
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
  return drake_ros_examples::do_main(argc, argv);
}
