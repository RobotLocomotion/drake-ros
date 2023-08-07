#include <filesystem>
#include <limits>
#include <memory>

#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake_ros/core/clock_system.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/tf2/scene_tf_broadcaster_system.h>
#include <drake_ros/viz/rviz_visualizer.h>
#include <gflags/gflags.h>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "How many seconds to run the simulation");

using drake_ros::core::ClockSystem;
using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;

using drake::systems::ConstantVectorSource;
using drake::systems::Simulator;
using drake::systems::TriggerType;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  // Create a Drake diagram
  drake::systems::DiagramBuilder<double> builder;

  // Initilise the ROS infrastructure
  drake_ros::core::init();
  // Create a Drake system to interface with ROS
  auto ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("multirobot_node"));
  ClockSystem::AddToBuilder(&builder,
                            ros_interface_system->get_ros_interface());

  // Add a multibody plant and a scene graph to hold the robots
  drake::multibody::MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.001;
  plant_config.discrete_contact_solver = "sap";
  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlant(plant_config, &builder);

  const double viz_dt = 1 / 32.0;
  // Add a TF2 broadcaster to provide task frame information
  auto scene_tf_broadcaster =
      builder.AddSystem<drake_ros::tf2::SceneTfBroadcasterSystem>(
          ros_interface_system->get_ros_interface(),
          drake_ros::tf2::SceneTfBroadcasterParams{
              {TriggerType::kPeriodic}, viz_dt, "/tf"});
  builder.Connect(scene_graph.get_query_output_port(),
                  scene_tf_broadcaster->get_graph_query_input_port());

  // Add a system to output the visualisation markers for rviz
  auto scene_visualizer = builder.AddSystem<drake_ros::viz::RvizVisualizer>(
      ros_interface_system->get_ros_interface(),
      drake_ros::viz::RvizVisualizerParams{
          {TriggerType::kPeriodic}, viz_dt, true});
  builder.Connect(scene_graph.get_query_output_port(),
                  scene_visualizer->get_graph_query_input_port());

  // Prepare to load the robot model
  auto parser = drake::multibody::Parser(&plant);
  auto model_file_path = drake::FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  const std::string model_name = "kuka_iiwa";

  // Create a 5x5 array of manipulators
  size_t kNumRows = 5;
  size_t kNumCols = 5;
  std::vector<std::vector<drake::multibody::ModelInstanceIndex>> models;
  for (size_t xx = 0; xx < kNumRows; ++xx) {
    std::vector<drake::multibody::ModelInstanceIndex> models_xx;
    for (size_t yy = 0; yy < kNumCols; ++yy) {
      // Load the model from the file and give it a name based on its X and Y
      // coordinates in the array
      std::stringstream model_instance_name;
      model_instance_name << model_name << xx << '_' << yy;
      auto model_instance =
          parser.AddModelFromFile(model_file_path, model_instance_name.str());

      // Weld the robot to the world so it doesn't fall through the floor
      auto& base_frame = plant.GetFrameByName("base", model_instance);
      auto X_WB = drake::math::RigidTransform(drake::Vector3<double>{
          static_cast<double>(xx), static_cast<double>(yy), 0.});
      plant.WeldFrames(plant.world_frame(), base_frame, X_WB);

      models_xx.push_back(model_instance);
    }
    models.push_back(models_xx);
  }

  // Finalise the multibody plant to make it ready for use
  plant.Finalize();

  // Set the control input of each robot to uncontrolled
  for (size_t xx = 0; xx < kNumRows; ++xx) {
    for (size_t yy = 0; yy < kNumCols; ++yy) {
      // Get the number of degrees of freedom for the robot
      auto num_dofs = plant.num_actuated_dofs(models[xx][yy]);
      // Create a vector with the same number of zeros
      auto u0 = drake::VectorX<double>::Zero(num_dofs);
      // Create a system that emits a constant value using that vector
      auto constant = builder.AddSystem<ConstantVectorSource>(u0);
      // Connect the constant value to the robot
      builder.Connect(constant->get_output_port(),
                      plant.get_actuation_input_port(models[xx][yy]));
    }
  }

  // Add a Drake visualiser instance to the diagram
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);

  // Build the complete system from the diagram
  auto diagram = builder.Build();

  // Create a simulator for the system
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  simulator->Initialize();
  auto& simulator_context = simulator->get_mutable_context();
  simulator->set_target_realtime_rate(1.0);

  // Step the simulator in 0.1s intervals
  constexpr double kStep{0.1};
  while (simulator_context.get_time() < FLAGS_simulation_sec) {
    const double next_time =
        std::min(FLAGS_simulation_sec, simulator_context.get_time() + kStep);
    simulator->AdvanceTo(next_time);
  }

  return 0;
}
