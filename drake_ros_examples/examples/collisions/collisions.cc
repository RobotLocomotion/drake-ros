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
 An example of visualizing ContactSurfaces with RViz.  */

#include <cmath>
#include <memory>
#include <utility>

#include "drake/multibody/plant/multibody_plant.h"
#include <drake/common/value.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/kinematics_vector.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/geometry/query_object.h>
#include <drake/geometry/query_results/contact_surface.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmt_contact_results_for_viz.hpp>
#include <drake/math/rigid_transform.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_interface_system.h>
#include <drake_ros_core/ros_publisher_system.h>
#include <drake_ros_viz/contact_markers_system.h>
#include <drake_ros_viz/rviz_visualizer.h>
#include <gflags/gflags.h>
#include <visualization_msgs/msg/marker_array.hpp>

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::AddRigidHydroelasticProperties;
using drake::geometry::HalfSpace;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_viz::ConnectContactResultsToRviz;
using drake_ros_viz::ContactMarkersParams;
using drake_ros_viz::RvizVisualizer;
using Eigen::Vector3d;
using Eigen::Vector4d;
using std::make_unique;

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_bool(real_time, true, "Set to false to run as fast as possible");
DEFINE_double(resolution_hint, 0.5,
              "Measure of typical mesh edge length in meters."
              " Smaller numbers produce a denser mesh");

int do_main() {
  DiagramBuilder<double> builder;

  drake_ros_core::init();
  auto ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("collisions"));

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  const double radius = 0.5;  // m
  const double mass = 10;     // kg
  const double g = 9.81;      // m/s^2

  const double hydroelastic_modulus = 1000;  // Pa
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
  SpatialInertia<double> M_Bcm(mass, Vector3d::Zero(), G_Bcm);

  const double alpha = 0.35;
  const Vector4d gray(0.5, 0.5, 0.5, alpha);
  const Vector4d orange(1.0, 0.55, 0.0, alpha);

  // Add a sloped half space
  RigidTransformd X_WG(RollPitchYawd(0.15, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));

  ProximityProperties ground_props;
  AddRigidHydroelasticProperties(&ground_props);
  AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                     &ground_props);
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, HalfSpace{},
                                  "collision_ground", std::move(ground_props));
  // Add visual for the ground.
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, HalfSpace{},
                               "visual_ground", gray);

  // Add a vertical wall (Q)
  RigidTransformd X_WQ(RollPitchYawd(-1.570796, 0.0, 0.0),
                       Vector3d(0.0, -10.0, 0.0));

  ProximityProperties wall_props;
  AddRigidHydroelasticProperties(&wall_props);
  AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                     &wall_props);
  plant.RegisterCollisionGeometry(plant.world_body(), X_WQ, HalfSpace{},
                                  "collision_wall", std::move(wall_props));
  // Add visual for the wall.
  plant.RegisterVisualGeometry(plant.world_body(), X_WQ, HalfSpace{},
                               "visual_wall", gray);

  const RigidBody<double>& compliant_ball =
      plant.AddRigidBody("CompliantBall", M_Bcm);
  const RigidBody<double>& rigid_ball = plant.AddRigidBody("RigidBall", M_Bcm);

  // Add sphere geometry for the balls.
  // Pose of sphere geometry S in body frame B.
  const RigidTransformd X_BS = RigidTransformd::Identity();
  // Set material properties for hydroelastics.
  {
    ProximityProperties ball_props;
    AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                       &ball_props);
    AddCompliantHydroelasticProperties(FLAGS_resolution_hint,
                                       hydroelastic_modulus, &ball_props);
    plant.RegisterCollisionGeometry(compliant_ball, X_BS, Sphere(radius),
                                    "collision", std::move(ball_props));

    plant.RegisterVisualGeometry(compliant_ball, X_BS, Sphere(radius), "visual",
                                 orange);
  }
  {
    ProximityProperties ball_props;
    AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                       &ball_props);
    AddRigidHydroelasticProperties(radius, &ball_props);
    plant.RegisterCollisionGeometry(rigid_ball, X_BS, Sphere(radius),
                                    "collision", std::move(ball_props));

    plant.RegisterVisualGeometry(rigid_ball, X_BS, Sphere(radius), "visual",
                                 orange);
  }

  // Gravity acting in the -z direction.
  plant.mutable_gravity_field().set_gravity_vector(-g * Vector3d::UnitZ());

  plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
  plant.Finalize();

  auto& rviz_visualizer = *builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());

  rviz_visualizer.RegisterMultibodyPlant(&plant);

  builder.Connect(scene_graph.get_query_output_port(),
                  rviz_visualizer.get_graph_query_port());

  ConnectContactResultsToRviz(&builder, plant, scene_graph,
                              ros_interface_system->get_ros_interface());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the initial poses for the balls.
  {
    RotationMatrixd R_WB(RollPitchYawd(M_PI / 180.0 * Vector3d(0.0, 0.0, 0.0)));
    RigidTransformd X_WB(R_WB, Vector3d(0.0, 0.0, 3.0));
    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("CompliantBall"),
                          X_WB);
  }

  {
    RotationMatrixd R_WB(RollPitchYawd(M_PI / 180.0 * Vector3d(0.0, 0.0, 0.0)));
    RigidTransformd X_WB(R_WB, Vector3d(0.0, radius * 4, 3.0));
    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("RigidBall"),
                          X_WB);
  }

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  auto& simulator_context = simulator.get_mutable_context();

  simulator.get_mutable_integrator().set_maximum_step_size(1.0 / 50.0);
  simulator.set_target_realtime_rate(FLAGS_real_time ? 1.f : 0.f);
  simulator.Initialize();

  while (true) {
    simulator.AdvanceTo(simulator_context.get_time() + 0.1);
  }
  simulator.AdvanceTo(FLAGS_simulation_time);
  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return do_main();
}
