// Copyright 2021 Open Source Robotics Foundation, Inc.
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

/** @file
 A simple binary for exercising and visualizing computation of ContactSurfaces.
 This is decoupled from dynamics so that just the geometric components can be
 evaluated in as light-weight a fashion as possible.
 This can serve as a test bed for evaluating the various cases of the
 ContactSurface-computing algorithms. Simply swap the geometry types (moving
 and anchored) and their properties to see the effect on contact surface.  */

/*
#include <drake/common/eigen_types.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/sine.h>
*/

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <drake/common/value.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/kinematics_vector.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/query_results/contact_surface.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/math/rigid_transform.h>
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_interface_system.h>
#include <drake_ros_core/ros_publisher_system.h>
#include <drake_ros_viz/rviz_visualizer.h>
#include <drake_ros_viz/contact_markers_system.h>
#include <gflags/gflags.h>

#include <visualization_msgs/msg/marker_array.hpp>

using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosPublisherSystem;
using drake_ros_viz::ContactMarkersParams;
using drake_ros_viz::ContactMarkersSystem;
using drake_ros_viz::RvizVisualizer;

namespace drake {
namespace examples {
namespace scene_graph {
namespace contact_surface {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::AddContactMaterial;
using geometry::AddRigidHydroelasticProperties;
using geometry::AddCompliantHydroelasticProperties;
using geometry::Box;
using geometry::ContactSurface;
using geometry::Cylinder;
using geometry::DrakeVisualizerd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::TriangleSurfaceMesh;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::ContactModel;
using multibody::CoulombFriction;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::SpatialVelocity;
using multibody::UnitInertia;
using std::make_unique;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::LeafSystem;
using systems::Simulator;
using systems::lcm::LcmPublisherSystem;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_double(resolution_hint, 1.0,
              "Measure of typical mesh edge length in meters."
              " Smaller numbers produce a denser mesh");
DEFINE_bool(rigid_cylinders, true,
            "Set to true, the cylinders are given a rigid "
            "hydroelastic representation");
DEFINE_bool(hybrid, false, "Set to true to run hybrid hydroelastic");


int do_main() {
  DiagramBuilder<double> builder;

  drake_ros_core::init();
  auto ros_interface_system =
      builder.AddSystem<RosInterfaceSystem>(
          std::make_unique<DrakeRos>("hydroelastic_collisions"));

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  const double radius = 0.1;   // m
  const double mass = 1;      // kg
  const double g = 9.81;        // m/s^2

  const double hydroelastic_modulus = 1e8;   // Pa
  const double dissipation = 5.0;            // s/m
  const double friction_coefficient = 0.3;

  const CoulombFriction<double> surface_friction(
      friction_coefficient /* static friction */,
      friction_coefficient /* dynamic friction */);

  // Hydroelastic Ball and Plane example taken from
  // https://github.com/RobotLocomotion/drake/blob/
  // 940b63716161c7206d494378701be11852c53d75/
  // examples/multibody/rolling_sphere/populate_ball_plant.cc
  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);


  // Add a sloped half space
  RigidTransformd X_WG(
    RollPitchYaw<double>(0.15, 0.0, 0.0),
    Vector3<double>(0.0, 0.0, 0.0));

  ProximityProperties ground_props;
  AddRigidHydroelasticProperties(&ground_props);
  AddContactMaterial(
      dissipation, {} /* point stiffness */, surface_friction, &ground_props);
  plant.RegisterCollisionGeometry(
      plant.world_body(), X_WG, geometry::HalfSpace{}, "collision",
      std::move(ground_props));
  // Add visual for the ground.
  plant.RegisterVisualGeometry(plant.world_body(), X_WG,
                                geometry::HalfSpace{}, "visual");

  const RigidBody<double>& ball = plant.AddRigidBody("Ball", M_Bcm);

  // Add sphere geometry for the ball.
  // Pose of sphere geometry S in body frame B.
  const RigidTransformd X_BS = RigidTransformd::Identity();
  // Set material properties for hydroelastics.
  ProximityProperties ball_props;
  AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                     &ball_props);
  AddCompliantHydroelasticProperties(radius, hydroelastic_modulus, &ball_props);
  plant.RegisterCollisionGeometry(ball, X_BS, Sphere(radius), "collision",
                                   std::move(ball_props));

  const double alpha = 0.35;
  // Add visual for the ball.
  const Vector4<double> orange(1.0, 0.55, 0.0, alpha);
  plant.RegisterVisualGeometry(
    ball, X_BS, Sphere(radius), "visual", orange);

  // Gravity acting in the -z direction.
  plant.mutable_gravity_field().set_gravity_vector(-g * Vector3d::UnitZ());

  plant.set_contact_model(ContactModel::kHydroelastic);
  plant.Finalize();

  auto& rviz_visualizer = *builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());

  rviz_visualizer.RegisterMultibodyPlant(&plant);

  // TODO(sloretz) make ConnectContactResultsToRviz() that does this
  // Publisher system for marker message
  auto hydroelastic_contact_markers_publisher = builder.AddSystem(
      RosPublisherSystem::Make<visualization_msgs::msg::MarkerArray>(
          "/hydroelastic_contact/mesh", rclcpp::QoS(1),
          ros_interface_system->get_ros_interface()));

  // System that turns contact results into ROS Messages
  auto hydroelastic_contact_markers = builder.AddSystem<ContactMarkersSystem>(
      plant, scene_graph, ContactMarkersParams::Strict());

  // TODO(sloretz) Drake quivalent seems to use Multibody plant instead of scene graph
  builder.Connect(
      scene_graph.get_query_output_port(),
      hydroelastic_contact_markers->get_graph_query_port());

  builder.Connect(
      hydroelastic_contact_markers->get_markers_output_port(),
      hydroelastic_contact_markers_publisher->get_input_port());

  // TODO(sloretz) where to get a multibody plant from?
  // hydroelastic_contact_markers->RegisterMultibodyPlant(plant);

  builder.Connect(scene_graph.get_query_output_port(),
                  rviz_visualizer.get_graph_query_port());

  // // Now visualize.
  // DrakeLcm lcm;

  // // Visualize geometry.
  // DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm);

  // TODO Use ConnectContactResultsToDrakeVisualizer() for comparison

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the sphere's initial pose.
  math::RotationMatrixd R_WB(math::RollPitchYawd(
      M_PI / 180.0 * Vector3d(0.0, 0.0, 0.0)));
  math::RigidTransformd X_WB(
      R_WB, 
      Vector3d(0.0, 0.0, 3.0));
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"), X_WB);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  auto & simulator_context = simulator.get_mutable_context();

  simulator.get_mutable_integrator().set_maximum_step_size(0.002);
  simulator.set_target_realtime_rate(FLAGS_real_time ? 1.f : 0.f);
  simulator.Initialize();

  while (true) {
    simulator.AdvanceTo(simulator_context.get_time() + 0.1);
  }
  simulator.AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace contact_surface
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::contact_surface::do_main();
}
