#!/usr/bin/env python3

import numpy as np
import sys
import os
import argparse
from PIL import Image
import shutil
from lxml import etree

from pydrake.all import (
    FindResourceOrThrow,
    Parser,
    AddMultibodyPlantSceneGraph,
    ConnectMeshcatVisualizer,
    DiagramBuilder,
    JacobianWrtVariable,
    Simulator,
)
from pydrake.geometry.render import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    RenderCameraCore,
    RenderLabel,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
)
from pydrake.geometry import (
    DrakeVisualizer,
    HalfSpace,
    FrameId,
    GeometrySet,
    CollisionFilterDeclaration,
)
from pydrake.systems.sensors import (
    CameraInfo,
    RgbdSensor,
)
from pydrake.math import RigidTransform, RollPitchYaw
import pydrake.multibody as mb
import multibody_extras as me


def xyz_rpy_deg(xyz, rpy_deg):
    # Shorthand for defining a pose.
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)


def make_parser(plant):
    parser = Parser(plant)
    parser.package_map().PopulateFromFolder("./repos/")
    return parser


def create_camera(builder, world_id, X_WB, depth_camera, scene_graph):
    sensor = RgbdSensor(world_id, X_PB=X_WB, depth_camera=depth_camera)
    builder.AddSystem(sensor)
    builder.Connect(
        scene_graph.get_query_output_port(), sensor.query_object_input_port()
    )
    return sensor


def infer_mask(image, bg_pixel=[255, 255, 255]):
    image_array = np.array(image)
    mask_bg1 = np.all(image_array == np.full(image_array.shape, bg_pixel), axis=2)
    return ~mask_bg1


def intersection_over_union(mask_a, mask_b):
    intersection = np.logical_and(mask_a, mask_b).sum()
    union = np.logical_or(mask_a, mask_b).sum()
    print(intersection / union)


def generate_images_and_iou(simulator, sensor, temp_directory, poses_dir, num_image):

    context = simulator.get_context()
    sensor_context = sensor.GetMyMutableContextFromRoot(context)

    color = sensor.color_image_output_port().Eval(sensor_context).data
    image_drake = Image.fromarray(color, "RGBA")

    image_drake.save(
        temp_directory + "/pics/" + poses_dir + "/" + str(num_image) + "_drake.png"
    )
    with Image.open(
        temp_directory + "/pics/" + poses_dir + "/" + str(num_image) + ".png"
    ) as image_ignition:
        mask_a = infer_mask(image_drake, image_drake.getpixel((0, 0)))
        mask_b = infer_mask(image_ignition, image_ignition.getpixel((0, 0)))
        intersection_over_union(mask_a, mask_b)


def remove_tag(tag, current):
    for element in current.findall(tag):
        current.remove(element)
    for element in list(current):
        remove_tag(tag, element)


def generate_sdf(model, poses_file, random, file_name):
    sdf_text = f"""<sdf version="1.6">
  <world name="default">
      <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
      </plugin>
      <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
      <background_color>0, 1, 0</background_color>
      </plugin>
      <plugin
          filename="ignition-gazebo-user-commands-system"
          name="ignition::gazebo::systems::UserCommands">
      </plugin>
      <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
      </plugin>
      <include>
      <uri>{model}</uri>
      <plugin
          filename="ignition-gazebo-model-photo-shoot-system"
          name="ignition::gazebo::systems::ModelPhotoShoot">
          <translation_data_file>{poses_file}</translation_data_file>
          <random_joints_pose>{random}</random_joints_pose>
      </plugin>
      </include>
      <model name="photo_shoot">
      <pose>2.2 0 0 0 0 -3.14</pose>
      <link name="link">
          <pose>0 0 0 0 0 0</pose>
          <sensor name="camera" type="camera">
          <camera>
              <horizontal_fov>1.047</horizontal_fov>
              <image>
              <width>960</width>
              <height>540</height>
              </image>
              <clip>
              <near>0.1</near>
              <far>100</far>
              </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
          <topic>camera</topic>
          </sensor>
      </link>
      <static>true</static>
      </model>
  </world>
</sdf>"""
    with open(file_name, "w") as f:
        f.write(sdf_text)


def perform_iou_testing(model_file, test_specific_temp_directory, pose_directory):

    random_poses = {}
    # Read camera translation calculated and applied on gazebo
    # we read the random positions file as it contains everything:
    with open(
        test_specific_temp_directory + "/pics/" + pose_directory + "/poses.txt", "r"
    ) as datafile:
        for line in datafile:
            if line.startswith("Translation:"):
                line_split = line.split(" ")
                # we make the value negative since gazebo moved the robot
                # and in drakewe move the camera
                trans_x = float(line_split[1])
                trans_y = float(line_split[2])
                trans_z = float(line_split[3])
            elif line.startswith("Scaling:"):
                line_split = line.split(" ")
                scaling = float(line_split[1])
            else:
                line_split = line.split(" ")
                if line_split[1] == "nan":
                    random_poses[line_split[0][:-1]] = 0
                else:
                    random_poses[line_split[0][:-1]] = float(line_split[1])

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)

    parser = Parser(plant)
    model = make_parser(plant).AddModelFromFile(model_file)

    model_bodies = me.get_bodies(plant, {model})
    frame_W = plant.world_frame()
    frame_B = model_bodies[0].body_frame()
    if len(plant.GetBodiesWeldedTo(plant.world_body())) < 2:
        plant.WeldFrames(
            frame_W, frame_B, X_PC=plant.GetDefaultFreeBodyPose(frame_B.body())
        )

    # Creating cameras:
    renderer_name = "renderer"
    scene_graph.AddRenderer(renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams()))

    # N.B. These properties are chosen arbitrarily.
    depth_camera = DepthRenderCamera(
        RenderCameraCore(
            renderer_name,
            CameraInfo(
                width=960,
                height=540,
                focal_x=831.382036787,
                focal_y=831.382036787,
                center_x=480,
                center_y=270,
            ),
            ClippingRange(0.01, 10.0),
            RigidTransform(),
        ),
        DepthRange(0.01, 10.0),
    )

    world_id = plant.GetBodyFrameIdOrThrow(plant.world_body().index())

    # Creating perspective cam
    X_WB = xyz_rpy_deg(
        [1.6 / scaling + trans_x, -1.6 / scaling + trans_y, 1.2 / scaling + trans_z],
        [-120, 0, 45],
    )
    sensor_perspective = create_camera(
        builder, world_id, X_WB, depth_camera, scene_graph
    )
    # Creating top cam
    X_WB = xyz_rpy_deg(
        [0 + trans_x, 0 + trans_y, 2.2 / scaling + trans_z], [-180, 0, -90]
    )
    sensor_top = create_camera(builder, world_id, X_WB, depth_camera, scene_graph)
    # Creating front cam
    X_WB = xyz_rpy_deg(
        [2.2 / scaling + trans_x, 0 + trans_y, 0 + trans_z], [-90, 0, 90]
    )
    sensor_front = create_camera(builder, world_id, X_WB, depth_camera, scene_graph)
    # Creating side cam
    X_WB = xyz_rpy_deg(
        [0 + trans_x, 2.2 / scaling + trans_y, 0 + trans_z], [-90, 0, 180]
    )
    sensor_side = create_camera(builder, world_id, X_WB, depth_camera, scene_graph)
    # Creating back cam
    X_WB = xyz_rpy_deg(
        [-2.2 / scaling + trans_x, 0 + trans_y, 0 + trans_z], [-90, 0, -90]
    )
    sensor_back = create_camera(builder, world_id, X_WB, depth_camera, scene_graph)

    DrakeVisualizer.AddToBuilder(builder, scene_graph)

    # Remove gravity to avoid extra movements of the model when running the simulation
    plant.gravity_field().set_gravity_vector(np.array([0, 0, 0], dtype=np.float64))

    # Switch off collisions to avoid problems with random positions
    collision_filter_manager = scene_graph.collision_filter_manager()
    model_inspector = scene_graph.model_inspector()
    geometry_ids = GeometrySet(model_inspector.GetAllGeometryIds())
    collision_filter_manager.Apply(
        CollisionFilterDeclaration().ExcludeWithin(geometry_ids)
    )

    plant.Finalize()
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()

    dofs = plant.num_actuated_dofs()
    if dofs != plant.num_positions():
        raise ValueError(
            "Error on converted model: Num positions is not equal to num actuated dofs."
        )

    if pose_directory == "random_pose":
        joint_positions = [0] * dofs
        for joint_name, pose in random_poses.items():
            # check if NaN
            if pose != pose:
                pose = 0
            # drake will add '_joint' when there's a name collision
            if plant.HasJointNamed(joint_name):
                joint = plant.GetJointByName(joint_name)
            else:
                joint = plant.GetJointByName(joint_name + "_joint")
            joint_positions[joint.position_start()] = pose
        sim_plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
        plant.get_actuation_input_port(model).FixValue(
            sim_plant_context, np.zeros((dofs, 1))
        )
        plant.SetPositions(sim_plant_context, model, joint_positions)

        simulator.AdvanceTo(1)

    generate_images_and_iou(
        simulator, sensor_perspective, test_specific_temp_directory, pose_directory, 1
    )
    generate_images_and_iou(
        simulator, sensor_top, test_specific_temp_directory, pose_directory, 2
    )
    generate_images_and_iou(
        simulator, sensor_front, test_specific_temp_directory, pose_directory, 3
    )
    generate_images_and_iou(
        simulator, sensor_side, test_specific_temp_directory, pose_directory, 4
    )
    generate_images_and_iou(
        simulator, sensor_back, test_specific_temp_directory, pose_directory, 5
    )


def setup_temporal_model_description_file(
    model_directory, description_file, temp_directory, mesh_type
):
    # Setup model temporal files
    temp_test_model_path = temp_directory + "/" + mesh_type + "/model/"
    model_file_path = temp_test_model_path + "/" + description_file
    shutil.copytree(model_directory, temp_test_model_path)
    root = etree.parse(model_file_path)
    model_name = root.find("model").attrib["name"]
    for uri in root.findall(".//uri"):
        uri.text = uri.text.replace("model://" + model_name + "/", "")

    if mesh_type == "collision":
        # Create ignore namespace so lxml don't complain
        # os.system("sed -i 's/visual/ignore:collision/g' " + model_file_path)
        my_namespaces = {"ignore": "http://ignore"}
        namespace = etree.Element("namespace", nsmap=my_namespaces)
        namespace.append(root.getroot())
        collision_tags = root.findall(".//collision")
        visual_tags = root.findall(".//visual")
        for collision_tag in collision_tags:
            collision_tag.tag = "visual"
        for visual_tag in visual_tags:
            visual_tag.tag = "{%s}visual" % my_namespaces["ignore"]

    data = etree.tostring(root, pretty_print=True).decode("utf-8")
    text_file = open(model_file_path, "w")
    text_file.write(data)
    text_file.close()

    return model_file_path


def run_test(
    model_file_path,
    temp_directory,
    mesh_type,
    type_joint_positions,
    poses_filename="poses.txt",
):
    # Setup temporal pics and metadata directory
    temp_default_pics_path = (
        temp_directory + "/" + mesh_type + "/pics/" + type_joint_positions + "/"
    )
    os.makedirs(temp_default_pics_path)
    current_dir = os.getcwd()
    os.chdir(temp_default_pics_path)
    plugin_config_path = temp_default_pics_path + "plugin_config.sdf"
    if type_joint_positions == "default_pose":
        generate_sdf(
            model_file_path,
            temp_default_pics_path + poses_filename,
            "false",
            plugin_config_path,
        )
    else:
        generate_sdf(
            model_file_path,
            temp_default_pics_path + poses_filename,
            "true",
            plugin_config_path,
        )

    os.system("ign gazebo -s -r --headless-rendering " + plugin_config_path + " --iterations 50")
    perform_iou_testing(
        model_file_path, temp_directory + "/" + mesh_type, type_joint_positions
    )
    os.chdir(current_dir)


def main(original_model_directory, description_file, temp_directory):

    mesh_type = "visual"
    tmp_model_file_path = setup_temporal_model_description_file(
        original_model_directory, description_file, temp_directory, mesh_type
    )
    run_test(tmp_model_file_path, temp_directory, mesh_type, "default_pose")
    run_test(tmp_model_file_path, temp_directory, mesh_type, "random_pose")

    mesh_type = "collision"
    tmp_model_file_path = setup_temporal_model_description_file(
        original_model_directory, description_file, temp_directory, mesh_type
    )
    run_test(tmp_model_file_path, temp_directory, mesh_type, "default_pose")
    run_test(tmp_model_file_path, temp_directory, mesh_type, "random_pose")


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "model_directory", help="Directory location of the model files"
        )
        parser.add_argument("description_file", help="Model description file name")
        parser.add_argument(
            "temp_directory",
            help="Temporal directory file where temporal objects will be written",
        )
        args = parser.parse_args()
        main(args.model_directory, args.description_file, args.temp_directory)
        print()
        print("[ Done ]")
    except UserError as e:
        eprint(e)
        sys.exit(1)
