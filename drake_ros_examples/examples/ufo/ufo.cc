#include <filesystem>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/geometry_conversions.h>
#include <drake_ros_core/ros_interface_system.h>
#include <drake_ros_core/ros_subscriber_system.h>
#include <drake_ros_tf2/scene_tf_broadcaster_system.h>
#include <drake_ros_viz/rviz_visualizer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::Parser;
using drake::systems::AbstractStateIndex;
using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_core::RosSubscriberSystem;
using drake_ros_tf2::SceneTfBroadcasterSystem;
using drake_ros_viz::RvizVisualizer;
using Eigen::Quaterniond;
using Eigen::Vector3d;

using Bodyd = drake::multibody::Body<double>;
using Contextd = drake::systems::Context<double>;
using DiagramBuilderd = drake::systems::DiagramBuilder<double>;
using Diagramd = drake::systems::Diagram<double>;
using ExternallyAppliedSpatialForced =
    drake::multibody::ExternallyAppliedSpatialForce<double>;
using LeafSystemd = drake::systems::LeafSystem<double>;
using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;
using RigidTransformd = drake::math::RigidTransform<double>;
using Simulatord = drake::systems::Simulator<double>;
using SpatialForced = drake::multibody::SpatialForce<double>;
using Stated = drake::systems::State<double>;

/// Adds UFO scene to the multibody plant.
/// \return index of flying saucer
ModelInstanceIndex AddUfoScene(MultibodyPlantd* plant) {
  auto parser = Parser(plant);
  parser.package_map().Add(
      "drake_ros_examples",
      ament_index_cpp::get_package_share_directory("drake_ros_examples"));

  // TODO(sloretz) make Drake Parser support package://
  std::filesystem::path ufo_path{
      parser.package_map().GetPath("drake_ros_examples")};
  parser.AddAllModelsFromFile((ufo_path / "models/ufo_scene.sdf").string());

  return plant->GetModelInstanceByName("spacecraft");
}

class FlyingSaucerController : public LeafSystemd {
 public:
  FlyingSaucerController(double saucer_mass, Vector3d gravity, Vector3d kp,
                         Vector3d kd, Vector3d kp_rot, Vector3d kd_rot,
                         double period = 1.0 / 100.0)
      : saucer_mass_(saucer_mass),
        gravity_(gravity),
        kp_(kp),
        kd_(kd),
        kp_rot_(kp_rot),
        kd_rot_(kd_rot),
        period_(period) {
    DeclareAbstractInputPort(kCurrentPosePort,
                             *drake::AbstractValue::Make(RigidTransformd()));
    DeclareAbstractInputPort(kTargetPosePort,
                             *drake::AbstractValue::Make(RigidTransformd()));

    DeclareAbstractOutputPort(kSpatialForcePort,
                              &FlyingSaucerController::CalcSpatialForce);

    X_WS_idx_ =
        DeclareAbstractState(*drake::AbstractValue::Make(RigidTransformd()));
    prev_X_WS_idx_ =
        DeclareAbstractState(*drake::AbstractValue::Make(RigidTransformd()));

    DeclarePeriodicUnrestrictedUpdateEvent(
        period_, 0., &FlyingSaucerController::UpdateState);
  }

  // Pose of saucer in world frame
  static constexpr const char* kCurrentPosePort{"X_WS"};
  // Target saucer pose in world frame
  static constexpr const char* kTargetPosePort{"X_WT"};
  // SpatialForce to be applied on saucer in world frame
  static constexpr const char* kSpatialForcePort{"F_S_W"};

 private:
  void UpdateState(const Contextd& context, Stated* state) const {
    auto& current_pose_port = GetInputPort(kCurrentPosePort);

    // Store current pose as old pose
    state->get_mutable_abstract_state<RigidTransformd>(prev_X_WS_idx_) =
        state->get_mutable_abstract_state<RigidTransformd>(X_WS_idx_);

    // TODO(sloretz) what happens if UpdateState is called faster than
    // X_WS input?
    // Store input pose as current pose
    state->get_mutable_abstract_state<RigidTransformd>(X_WS_idx_) =
        current_pose_port.Eval<RigidTransformd>(context);
  }

  void CalcSpatialForce(const Contextd& context, SpatialForced* output) const {
    // Get target pose
    auto& target_pose_port = GetInputPort(kTargetPosePort);
    const auto& X_WT = target_pose_port.Eval<RigidTransformd>(context);

    // Get Current and previous pose
    const auto& avs = context.get_abstract_state();
    const auto& X_WS = avs.get_value(X_WS_idx_).get_value<RigidTransformd>();
    const auto& prev_X_WS =
        avs.get_value(prev_X_WS_idx_).get_value<RigidTransformd>();

    // force to be applied to saucer in world frame
    Vector3d f_S_W{0, 0, 0};
    // torque to be applied to saucer in world frame
    Vector3d t_S_W{0, 0, 0};

    // Translation
    // p_WS = Current saucer position in world frame
    // p_WT = Target saucer position in world frame
    const auto& p_WS = X_WS.translation();
    const auto& p_WT = X_WT.translation();

    // Translation Error
    const auto p_error = p_WT - p_WS;
    const auto prev_p_error = p_WT - prev_X_WS.translation();

    // Translation PD controller - proportional
    f_S_W += ((p_error).array() * kp_.array()).matrix();

    // Translation PD controller - derivative
    const auto de_dt = (p_error - prev_p_error) / period_;
    f_S_W += (de_dt.array() * kd_.array()).matrix();

    // Translation PD Gravity feedforward
    f_S_W -= saucer_mass_ * gravity_;

    // Orientation
    const auto& R_WS = X_WS.rotation();
    const auto& R_WT = X_WT.rotation();

    // Rotation from current to target orientation
    const auto R_error = R_WT * R_WS.inverse();
    const auto prev_R_error = R_WT * prev_X_WS.rotation().inverse();

    // Orientation PD controller - proportional
    t_S_W +=
        (R_error.ToRollPitchYaw().vector().array() * kp_rot_.array()).matrix();

    // Orientation PD controller - derivative
    const auto rot_de_dt =
        (R_error * prev_R_error.inverse()).ToRollPitchYaw().vector() / period_;
    t_S_W += (rot_de_dt.array() * kd_rot_.array()).matrix();

    *output = SpatialForced(t_S_W, f_S_W);
  }

  const double saucer_mass_;
  const Vector3d gravity_;
  const Vector3d kp_;
  const Vector3d kd_;
  const Vector3d kp_rot_;
  const Vector3d kd_rot_;
  const double period_;

  AbstractStateIndex X_WS_idx_;
  AbstractStateIndex prev_X_WS_idx_;
};

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

class RigidTransformPremultiplier : public LeafSystemd {
 public:
  explicit RigidTransformPremultiplier(const RigidTransformd& X_BC) : X_BC_(X_BC) {
    DeclareAbstractInputPort(
        kInputPort,
        *drake::AbstractValue::Make(RigidTransformd()));

    DeclareAbstractOutputPort(kOutputPort, &RigidTransformPremultiplier::CalcTransform);
  }

  static constexpr const char* kInputPort{"X_AB"};
  static constexpr const char* kOutputPort{"X_AC"};
 protected:
  void CalcTransform(const Contextd& context,
                          RigidTransformd* X_AC) const {
    auto& input_port = GetInputPort(kInputPort);
    auto& X_AB = input_port.Eval<RigidTransformd>(context);
    *X_AC = X_AB * X_BC_;
  }

  const RigidTransformd X_BC_;
};

class RosPoseGlue : public LeafSystemd {
 public:
  explicit RosPoseGlue() {
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

    *output = drake_ros_core::RosPoseToRigidTransform(goal_pose.pose);
  }
};

/// Build a simulation and set initial conditions.
std::unique_ptr<Diagramd> BuildSimulation() {
  DiagramBuilderd builder;

  auto [plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  ModelInstanceIndex saucer_idx = AddUfoScene(&plant);

  plant.Finalize();

  double saucer_mass{0.0};
  auto body_idxs = plant.GetBodyIndices(saucer_idx);
  for (auto& body_idx : body_idxs) {
    const Bodyd& body = plant.get_body(body_idx);
    saucer_mass += body.default_mass();
  }

  auto ufo_controller = builder.AddSystem<FlyingSaucerController>(
      saucer_mass, plant.gravity_field().gravity_vector(),
      Vector3d{100.0, 100.0, 100.0},  // kp & kd chosen by trial and error
      Vector3d{500.0, 500.0, 400.0},
      Vector3d{1000.0, 1000.0, 1000.0},  // kp_rot & kd_rot also trial and error
      Vector3d{1000.0, 1000.0, 1000.0});

  // Glue controller to multibody plant
  // Get saucer poses X_WS to controller
  const BodyIndex ufo_index = plant.GetBodyByName("spacecraft").index();
  auto* body_pose_at_index = builder.AddSystem<BodyPoseAtIndex>(ufo_index);
  builder.Connect(
      plant.get_body_poses_output_port(),
      body_pose_at_index->GetInputPort(BodyPoseAtIndex::kBodyPosesPort));
  builder.Connect(body_pose_at_index->GetOutputPort(BodyPoseAtIndex::kPosePort),
                  ufo_controller->GetInputPort("X_WS"));

  // Connect UFO Controller's output to applied force accepted by MultibodyPlant
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
  auto goal_glue = builder.AddSystem<RosPoseGlue>();
  builder.Connect(goal_sub->get_output_port(), goal_glue->get_input_port());

  // RViz goal is on the ground - add system to raise it by 10 meters
  auto goal_offsetter = builder.AddSystem<RigidTransformPremultiplier>(
      RigidTransformd(Vector3d{0, 0, 10.0}));
  builder.Connect(goal_glue->get_output_port(),
                  goal_offsetter->get_input_port());
  builder.Connect(goal_offsetter->get_output_port(),
                  ufo_controller->GetInputPort("X_WT"));

  return builder.Build();
}

int main() {
  drake_ros_core::init();

  std::unique_ptr<Diagramd> diagram = BuildSimulation();
  std::unique_ptr<Contextd> diagram_context = diagram->CreateDefaultContext();

  auto simulator =
      std::make_unique<Simulatord>(*diagram, std::move(diagram_context));

  Contextd& simulator_context = simulator->get_mutable_context();
  simulator->get_mutable_integrator().set_maximum_step_size(1.0 / 50.0);
  simulator->set_target_realtime_rate(1.0);

  simulator->Initialize();

  while (true) {
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
  }

  return 0;
}
