#include <filesystem>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/pid_controller.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_interface_system.h>
#include <drake_ros_core/ros_subscriber_system.h>
#include <drake_ros_tf2/scene_tf_broadcaster_system.h>
#include <drake_ros_viz/rviz_visualizer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using drake::multibody::BodyIndex;
using drake::multibody::Parser;
using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosSubscriberSystem;
using drake_ros_tf2::SceneTfBroadcasterSystem;
using drake_ros_viz::RvizVisualizer;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using BasicVectord = drake::systems::BasicVector<double>;
using ConstantVectorSourced = drake::systems::ConstantVectorSource<double>;
using Contextd = drake::systems::Context<double>;
using Diagramd = drake::systems::Diagram<double>;
using DiagramBuilderd = drake::systems::DiagramBuilder<double>;
using ExternallyAppliedSpatialForced =
    drake::multibody::ExternallyAppliedSpatialForce<double>;
using LeafSystemd = drake::systems::LeafSystem<double>;
using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;
using Multiplexerd = drake::systems::Multiplexer<double>;
using PidControllerd = drake::systems::controllers::PidController<double>;
using RigidTransformd = drake::math::RigidTransform<double>;
using RollPitchYawd = drake::math::RollPitchYaw<double>;
using Simulatord = drake::systems::Simulator<double>;
using SpatialForced = drake::multibody::SpatialForce<double>;
using StateInterpolatorWithDiscreteDerivatived =
    drake::systems::StateInterpolatorWithDiscreteDerivative<double>;
using Systemd = drake::systems::System<double>;

/// Adds body named FlyingSaucer to the multibody plant.
void AddFlyingSaucer(MultibodyPlantd* plant) {
  auto parser = Parser(plant);
  std::filesystem::path pkg_share_dir{
    ament_index_cpp::get_package_share_directory("drake_ros_examples")
  };
  const char * kUfoPath = "models/ufo.sdf";
  std::string model_file_path = (pkg_share_dir / kUfoPath).string();
  parser.AddModelFromFile(model_file_path, "spacecraft");
}

/// Adds Ground geometry to the world in the multibody plant.
void AddGround(MultibodyPlantd* plant) {
  auto parser = Parser(plant);
  std::filesystem::path pkg_share_dir{
    ament_index_cpp::get_package_share_directory("drake_ros_examples")
  };
  const char * kGroundPath = "models/ground.sdf";
  std::string model_file_path = (pkg_share_dir / kGroundPath).string();
  parser.AddModelFromFile(model_file_path, "ground");
}

class SplitRigidTransform : public LeafSystemd {
 public:
  SplitRigidTransform() {
    DeclareAbstractInputPort(kTransformPort,
                             *drake::AbstractValue::Make(RigidTransformd()));

    DeclareVectorOutputPort(kOrientationPort, BasicVectord(3),
                            &SplitRigidTransform::CalcOrientation);

    DeclareVectorOutputPort(kPositionPort, BasicVectord(3),
                            &SplitRigidTransform::CalcPosition);
  }

  virtual ~SplitRigidTransform() = default;

  static constexpr const char* kTransformPort{"X_WF"};
  static constexpr const char* kOrientationPort{"R_WF"};
  static constexpr const char* kPositionPort{"p_WF"};

 private:
  void CalcOrientation(const Contextd& context, BasicVectord* output) const {
    auto& transform_port = GetInputPort(kTransformPort);
    const auto& X_WF = transform_port.Eval<RigidTransformd>(context);
    output->SetFromVector(RollPitchYawd(X_WF.rotation()).vector());
  }

  void CalcPosition(const Contextd& context, BasicVectord* output) const {
    auto& transform_port = GetInputPort(kTransformPort);
    const auto& X_WF = transform_port.Eval<RigidTransformd>(context);
    output->SetFromVector(X_WF.translation());
  }
};

class UnsplitSpatialForce : public LeafSystemd {
 public:
  UnsplitSpatialForce() {
    DeclareVectorInputPort(kForcesPort, BasicVectord(3));
    DeclareVectorInputPort(kTorquesPort, BasicVectord(3));

    DeclareAbstractOutputPort(kSpatialForcePort,
                              &UnsplitSpatialForce::CalcSpatialForce);
  }

  virtual ~UnsplitSpatialForce() = default;

  static constexpr const char* kForcesPort{"f_F"};
  static constexpr const char* kTorquesPort{"t_F"};
  static constexpr const char* kSpatialForcePort{"F_F"};

 private:
  void CalcSpatialForce(const Contextd& context, SpatialForced* output) const {
    auto& forces_port = GetInputPort(kForcesPort);
    auto& torques_port = GetInputPort(kTorquesPort);

    const auto& f_F = forces_port.Eval<BasicVectord>(context).value();
    const auto& t_F = torques_port.Eval<BasicVectord>(context).value();
    *output = SpatialForced(t_F, f_F);
  }
};

std::unique_ptr<Diagramd> CreateSaucerController() {
  // X_WS = Pose of saucer in world frame
  // X_WT = Target saucer pose in world frame
  // p_WS = Current saucer position in world frame
  // p_WT = Target saucer position in world frame
  // f_S_W = force to be applied to saucer in world frame
  // t_S_W = torque to be applied to saucer in world frame
  // F_S_W = SpatialForce to be applied on saucer in world frame

  // TODO(eric.cousineau): Add orientation controller

  // Not included: glue to MultibodyPlant's vectors of stuff

  DiagramBuilderd builder;

  // Input glue (current pose splitter)
  // input: RigidTransform X_WS
  // output: Vector3d (Euler) R_WS
  // output: Vector3d p_WS
  auto* current_pose_glue = builder.AddSystem<SplitRigidTransform>();

  // Input glue (target pose splitter)
  // input: RigidTransform X_WT
  // output: Vector3d (Euler) R_WT
  // output: Vector3d p_WT
  auto* target_pose_glue = builder.AddSystem<SplitRigidTransform>();

  // Zero velocity for target pose
  auto* zero_vector3 =
      builder.AddSystem<ConstantVectorSourced>(Vector3d{0.0, 0.0, 0.0});

  // Current position state with derivative
  // input: p_WS
  // output: p_WS concatenated with v_WS
  auto* current_position_interp =
      builder.AddSystem<StateInterpolatorWithDiscreteDerivatived>(3, 0.01);
  builder.Connect(
      current_pose_glue->GetOutputPort(SplitRigidTransform::kPositionPort),
      current_position_interp->get_input_port());

  // Target position mux
  // input: p_WT
  // output: p_WT concatenated with v_WT (zeros)
  auto* target_position_mux =
      builder.AddSystem<Multiplexerd>(std::vector<int>{3, 3});
  builder.Connect(
      target_pose_glue->GetOutputPort(SplitRigidTransform::kPositionPort),
      target_position_mux->get_input_port(0));
  builder.Connect(zero_vector3->get_output_port(),
                  target_position_mux->get_input_port(1));

  // Forces PidController
  //  input: estimated state Vector3d p_WS concatenated with v_WS
  //  input: desired state Vector3d p_WT concatenated with v_WT
  //  output: Vector3d f_S_W
  auto* forces_pid_controller = builder.AddSystem<PidControllerd>(
      Vector3d{100.0f, 100.0f, 2500.0f}, Vector3d{0.0f, 0.0f, 50.0f},
      Vector3d{500.0f, 500.0f, 500.0f});
  builder.Connect(current_position_interp->get_output_port(),
                  forces_pid_controller->get_input_port_estimated_state());
  builder.Connect(target_position_mux->get_output_port(),
                  forces_pid_controller->get_input_port_desired_state());

  // Output Glue
  // input: Vector3d f_S_W
  // input: Vector3d t_S_W (zeros)
  // output: SpacialForce F_S_W
  auto* spatial_force_combiner = builder.AddSystem<UnsplitSpatialForce>();
  builder.Connect(
      forces_pid_controller->get_output_port_control(),
      spatial_force_combiner->GetInputPort(UnsplitSpatialForce::kForcesPort));
  builder.Connect(
      zero_vector3->get_output_port(),
      spatial_force_combiner->GetInputPort(UnsplitSpatialForce::kTorquesPort));

  // Whole diagram ports
  //  input: RigidTransform X_WS
  //  input: RigidTransform X_WT
  //  output: SpatialForced F_S_W
  builder.ExportInput(
      current_pose_glue->GetInputPort(SplitRigidTransform::kTransformPort),
      "X_WS");
  builder.ExportInput(
      target_pose_glue->GetInputPort(SplitRigidTransform::kTransformPort),
      "X_WT");
  builder.ExportOutput(spatial_force_combiner->GetOutputPort(
                           UnsplitSpatialForce::kSpatialForcePort),
                       "F_S_W");

  return builder.Build();
}

class BodyPoseAtIndex : public LeafSystemd {
 public:
  explicit BodyPoseAtIndex(BodyIndex index) : index_(index) {
    DeclareAbstractInputPort(
        kBodyPosesPort,
        *drake::AbstractValue::Make(std::vector<RigidTransformd>{}));

    DeclareAbstractOutputPort(kPosePort, &BodyPoseAtIndex::CalcPose);
  }

  virtual ~BodyPoseAtIndex() = default;

  static constexpr const char* kBodyPosesPort{"body_poses"};
  static constexpr const char* kPosePort{"pose"};

 private:
  void CalcPose(const Contextd& context, RigidTransformd* output) const {
    auto& body_poses_port = GetInputPort(kBodyPosesPort);

    const auto& body_poses =
        body_poses_port.Eval<std::vector<RigidTransformd>>(context);
    *output = body_poses.at(index_);
  }

  const BodyIndex index_;
};

class AppliedSpatialForceVector : public LeafSystemd {
 public:
  explicit AppliedSpatialForceVector(BodyIndex index) : index_(index) {
    DeclareAbstractInputPort(kSpatialForcePort,
                             *drake::AbstractValue::Make(SpatialForced()));

    DeclareAbstractOutputPort(
        kAppliedSpatialForcePort,
        &AppliedSpatialForceVector::CalcAppliedSpatialForceVector);
  }

  virtual ~AppliedSpatialForceVector() = default;

  static constexpr const char* kSpatialForcePort{"body_poses"};
  static constexpr const char* kAppliedSpatialForcePort{
      "applied_spatial_force"};

 private:
  void CalcAppliedSpatialForceVector(
      const Contextd& context,
      std::vector<ExternallyAppliedSpatialForced>* output) const {
    auto& input_port = GetInputPort(kSpatialForcePort);

    const auto& F_WB = input_port.Eval<SpatialForced>(context);
    ExternallyAppliedSpatialForced spatial_force;
    spatial_force.body_index = index_;
    spatial_force.p_BoBq_B = Vector3d{0, 0, 0};
    spatial_force.F_Bq_W = F_WB;
    output->clear();
    output->push_back(spatial_force);
  }

  const BodyIndex index_;
};

class RosPoseGlue : public LeafSystemd {
 public:
  explicit RosPoseGlue(double extra_z = 0.0) : extra_z_(extra_z) {
    DeclareAbstractInputPort(
        kRosMsgPort,
        *drake::AbstractValue::Make(geometry_msgs::msg::PoseStamped()));

    DeclareAbstractOutputPort(kOutputPort, &RosPoseGlue::CalcDrakeTransform);
  }

  virtual ~RosPoseGlue() = default;

  static constexpr const char* kRosMsgPort{"ros_pose"};
  static constexpr const char* kOutputPort{"drake_pose"};

 private:
  void CalcDrakeTransform(const Contextd& context,
                          RigidTransformd* output) const {
    auto& input_port = GetInputPort(kRosMsgPort);

    // TODO(sloretz) use RobotLocomotion/drake-ros#141
    const auto& goal_pose =
        input_port.Eval<geometry_msgs::msg::PoseStamped>(context);

    Vector3d translation{
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        goal_pose.pose.position.z + extra_z_,
    };
    Quaterniond rotation{
        goal_pose.pose.orientation.w,
        goal_pose.pose.orientation.x,
        goal_pose.pose.orientation.y,
        goal_pose.pose.orientation.z,
    };

    output->set_translation(translation);
    output->set_rotation(rotation);
  }

  const double extra_z_;
};

/// Build a simulation and set initial conditions.
std::unique_ptr<Diagramd> BuildSimulation() {
  DiagramBuilderd builder;

  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  AddGround(&plant);
  AddFlyingSaucer(&plant);

  plant.Finalize();

  auto* ufo_controller = builder.AddSystem(CreateSaucerController());

  // Glue controller to multibody plant
  // Get saucer poses X_WS to controller
  const BodyIndex ufo_index = plant.GetBodyByName("spacecraft").index();
  auto* body_pose_at_index = builder.AddSystem<BodyPoseAtIndex>(ufo_index);
  builder.Connect(
      plant.get_body_poses_output_port(),
      body_pose_at_index->GetInputPort(BodyPoseAtIndex::kBodyPosesPort));
  builder.Connect(body_pose_at_index->GetOutputPort(BodyPoseAtIndex::kPosePort),
                  ufo_controller->GetInputPort("X_WS"));

  // TODO(sloretz) replace when RobotLocomotion/drake#16923 is solved
  auto* apply_force_glue =
      builder.AddSystem<AppliedSpatialForceVector>(ufo_index);
  builder.Connect(ufo_controller->GetOutputPort("F_S_W"),
                  apply_force_glue->get_input_port());
  builder.Connect(apply_force_glue->get_output_port(),
                  plant.get_applied_spatial_force_input_port());

  // Add basic system to create a ROS node
  auto* ros_interface_system = builder.AddSystem<RosInterfaceSystem>(
      std::make_unique<DrakeRos>("systems_framework_demo"));

  // Add a TF2 broadcaster to provide frame info to ROS
  auto* scene_tf_broadcaster = builder.AddSystem<SceneTfBroadcasterSystem>(
      ros_interface_system->get_ros_interface());
  builder.Connect(scene_graph.get_query_output_port(),
                  scene_tf_broadcaster->get_graph_query_input_port());

  // Add a system to output the visualisation markers for RViz
  auto* scene_visualizer = builder.AddSystem<RvizVisualizer>(
      ros_interface_system->get_ros_interface());
  scene_visualizer->RegisterMultibodyPlant(&plant);
  builder.Connect(scene_graph.get_query_output_port(),
                  scene_visualizer->get_graph_query_input_port());

  // Add system to get goal pose from RViz
  auto* goal_sub = builder.AddSystem(
      RosSubscriberSystem::Make<geometry_msgs::msg::PoseStamped>(
          "goal_pose", rclcpp::QoS(1),
          ros_interface_system->get_ros_interface()));
  auto goal_glue = builder.AddSystem<RosPoseGlue>(10.0f);
  builder.Connect(goal_sub->get_output_port(), goal_glue->get_input_port());
  builder.Connect(goal_glue->get_output_port(),
                  ufo_controller->GetInputPort("X_WT"));

  return builder.Build();
}

std::unique_ptr<Contextd> SetInitialConditions(Diagramd* diagram) {
  std::unique_ptr<Contextd> diagram_context = diagram->CreateDefaultContext();

  const Systemd& plant_system = diagram->GetSubsystemByName("plant");
  const MultibodyPlantd& plant =
      dynamic_cast<const MultibodyPlantd&>(plant_system);
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  RigidTransformd X_WS(RollPitchYawd(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));
  plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("spacecraft"),
                        X_WS);
  return diagram_context;
}

void RunSimulation(Diagramd* diagram,
                   std::unique_ptr<Contextd> diagram_context) {
  auto simulator =
      std::make_unique<Simulatord>(*diagram, std::move(diagram_context));

  Contextd& simulator_context = simulator->get_mutable_context();
  simulator->get_mutable_integrator().set_maximum_step_size(1.0 / 50.0);
  simulator->set_target_realtime_rate(1.0);

  simulator->Initialize();

  while (true) {
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
  }
}

int main() {
  drake_ros_core::init();

  std::unique_ptr<Diagramd> diagram = BuildSimulation();
  std::unique_ptr<Contextd> diagram_context =
      SetInitialConditions(diagram.get());

  RunSimulation(diagram.get(), std::move(diagram_context));

  return 0;
}
