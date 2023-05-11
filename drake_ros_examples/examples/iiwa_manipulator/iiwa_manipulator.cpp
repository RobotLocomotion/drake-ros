#include <cmath>
#include <memory>
#include <utility>

#include <drake/common/eigen_types.h>
#include <drake/examples/manipulation_station/manipulation_station.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/sine.h>
#include <drake_ros/core/clock_system.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/viz/rviz_visualizer.h>
#include <gflags/gflags.h>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "How many seconds to run the simulation");

using drake_ros::core::ClockSystem;
using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::viz::RvizVisualizer;

using drake::examples::manipulation_station::ManipulationStation;
using drake::systems::Adder;
using drake::systems::ConstantVectorSource;
using drake::systems::Simulator;
using drake::systems::Sine;

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  drake::systems::DiagramBuilder<double> builder;

  drake_ros::core::init();
  auto ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("iiwa_manipulator_node"));
  ClockSystem::AddToBuilder(&builder,
                            ros_interface_system->get_ros_interface());

  auto manipulation_station = builder.AddSystem<ManipulationStation>();
  manipulation_station->SetupClutterClearingStation();
  manipulation_station->Finalize();

  // Make the base joint swing sinusoidally.
  auto constant_term = builder.AddSystem<ConstantVectorSource>(
      drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints()));

  drake::VectorX<double> amplitudes =
      drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints());
  amplitudes[0] = M_PI / 4.;  // == 45 degrees
  const drake::VectorX<double> frequencies = drake::VectorX<double>::Constant(
      manipulation_station->num_iiwa_joints(), 1.);  // Hz
  const drake::VectorX<double> phases =
      drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints());
  auto variable_term = builder.AddSystem<Sine>(amplitudes, frequencies, phases);

  auto joint_trajectory_generator =
      builder.AddSystem<Adder>(2, manipulation_station->num_iiwa_joints());

  builder.Connect(constant_term->get_output_port(),
                  joint_trajectory_generator->get_input_port(0));
  builder.Connect(variable_term->get_output_port(0),
                  joint_trajectory_generator->get_input_port(1));
  builder.Connect(joint_trajectory_generator->get_output_port(),
                  manipulation_station->GetInputPort("iiwa_position"));

  auto rviz_visualizer = builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());

  rviz_visualizer->RegisterMultibodyPlant(
      &manipulation_station->get_multibody_plant());
  rviz_visualizer->ComputeFrameHierarchy();

  builder.Connect(manipulation_station->GetOutputPort("query_object"),
                  rviz_visualizer->get_graph_query_input_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto& simulator_context = simulator->get_mutable_context();

  auto& manipulation_station_context = diagram->GetMutableSubsystemContext(
      *manipulation_station, &simulator_context);

  auto& constant_term_context =
      diagram->GetMutableSubsystemContext(*constant_term, &simulator_context);

  // Fix gripper joints' position.
  manipulation_station->GetInputPort("wsg_position")
      .FixValue(&manipulation_station_context, 0.);

  // Use default positions for every joint but the base joint.
  drake::systems::BasicVector<double>& constants =
      constant_term->get_mutable_source_value(&constant_term_context);
  constants.set_value(
      manipulation_station->GetIiwaPosition(manipulation_station_context));
  constants.get_mutable_value()[0] = -M_PI / 4.;

  // Step the simulator in 0.1s intervals
  constexpr double kStep{0.1};
  while (simulator_context.get_time() < FLAGS_simulation_sec) {
    const double next_time =
        std::min(FLAGS_simulation_sec, simulator_context.get_time() + kStep);
    simulator->AdvanceTo(next_time);
  }
  return 0;
}
