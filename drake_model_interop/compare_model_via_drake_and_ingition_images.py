#!/usr/bin/env python3

import numpy as np
import sys
import os
import subprocess
import argparse
from PIL import Image
import shutil
from lxml import etree
import textwrap
import math

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import DrakeVisualizer

from pydrake.geometry.render import (
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    RenderCameraCore,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
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
    return RigidTransform(RollPitchYaw(np.deg2rad(rpy_deg)), xyz)


def make_parser(plant, package_dir):
    parser = Parser(plant)
    parser.package_map().PopulateFromFolder(package_dir)
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
    background_mask = np.all(
        image_array == np.full(image_array.shape, bg_pixel), axis=2
    )
    return ~background_mask


def intersection_over_union(mask_a, mask_b, threshold):
    intersection = np.logical_and(mask_a, mask_b).sum()
    union = np.logical_or(mask_a, mask_b).sum()
    iou_result = intersection / union
    if iou_result < threshold:
        raise ValueError(
            f"Intersection over union test value {iou_result} was lower than threshold: {threshold}."
        )
    print(iou_result)


def generate_images_and_iou(
    sensor_context, sensor, temp_directory, poses_dir, num_image, iou_test_threshold
):

    color = sensor.color_image_output_port().Eval(sensor_context).data
    image_drake = Image.fromarray(color, "RGBA")

    image_drake.save(
        os.path.join(temp_directory, "pics", poses_dir, f"{num_image}_drake.png")
    )
    with Image.open(
        os.path.join(temp_directory, "pics", poses_dir, f"{num_image}.png")
    ) as image_ignition:
        mask_a = infer_mask(image_drake, image_drake.getpixel((0, 0)))
        mask_b = infer_mask(image_ignition, image_ignition.getpixel((0, 0)))
        intersection_over_union(mask_a, mask_b, iou_test_threshold)


def remove_tag(tag, current):
    for element in current.findall(tag):
        current.remove(element)
    for element in list(current):
        remove_tag(tag, element)


def generate_sdf(model, poses_file, random, file_name, render_camera_core):
    sdf_text = textwrap.dedent(
        f"""\
<sdf version="1.9">
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
            <pose>2.2 0 0 0 0 {math.pi}</pose>
            <link name="link">
                <pose>0 0 0 0 0 0</pose>
                <sensor name="camera" type="camera">
                    <camera>
                        <horizontal_fov>
                                {2*math.atan(render_camera_core.intrinsics().width()/
                                             (2*render_camera_core.intrinsics().focal_x()))}
                        </horizontal_fov>
                        <image>
                            <width>{render_camera_core.intrinsics().width()}</width>
                            <height>{render_camera_core.intrinsics().height()}</height>
                        </image>
                        <clip>
                            <near>{render_camera_core.clipping().near()}</near>
                            <far>{render_camera_core.clipping().far()}</far>
                        </clip>
                    </camera>
                    <always_on>1</always_on>
                    <visualize>true</visualize>
                    <topic>camera</topic>
                </sensor>
            </link>
            <static>true</static>
        </model>
    </world>
</sdf>"""
    )
    with open(file_name, "w") as f:
        f.write(sdf_text)


def perform_iou_testing(
    model_file,
    test_specific_temp_directory,
    pose_directory,
    randomize_poses,
    render_camera_core,
    iou_test_threshold,
    drake_visualizer,
):

    random_poses = {}
    # Read camera translation calculated and applied on gazebo
    # we read the random positions file as it contains everything:
    with open(
        os.path.join(test_specific_temp_directory, "pics", pose_directory, "poses.txt"),
        "r",
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
    model = make_parser(plant, test_specific_temp_directory).AddModelFromFile(
        model_file
    )

    model_bodies = me.get_bodies(plant, {model})
    frame_W = plant.world_frame()
    frame_B = model_bodies[0].body_frame()
    if len(plant.GetBodiesWeldedTo(plant.world_body())) < 2:
        plant.WeldFrames(
            frame_W, frame_B, X_PC=plant.GetDefaultFreeBodyPose(frame_B.body())
        )

    # Creating cameras:
    scene_graph.AddRenderer(
        render_camera_core.renderer_name(), MakeRenderEngineVtk(RenderEngineVtkParams())
    )

    # N.B. These properties are chosen arbitrarily.
    depth_camera = DepthRenderCamera(
        render_camera_core,
        DepthRange(0.01, 10.0),
    )

    world_id = plant.GetBodyFrameIdOrThrow(plant.world_body().index())

    # Creating perspective cam
    sensors = []
    X_WB = xyz_rpy_deg(
        [1.6 / scaling + trans_x, -1.6 / scaling + trans_y, 1.2 / scaling + trans_z],
        [-120, 0, 45],
    )
    sensors.append(create_camera(builder, world_id, X_WB, depth_camera, scene_graph))
    # Creating top cam
    X_WB = xyz_rpy_deg(
        [0 + trans_x, 0 + trans_y, 2.2 / scaling + trans_z], [-180, 0, -90]
    )
    sensors.append(create_camera(builder, world_id, X_WB, depth_camera, scene_graph))
    # Creating front cam
    X_WB = xyz_rpy_deg(
        [2.2 / scaling + trans_x, 0 + trans_y, 0 + trans_z], [-90, 0, 90]
    )
    sensors.append(create_camera(builder, world_id, X_WB, depth_camera, scene_graph))
    # Creating side cam
    X_WB = xyz_rpy_deg(
        [0 + trans_x, 2.2 / scaling + trans_y, 0 + trans_z], [-90, 0, 180]
    )
    sensors.append(create_camera(builder, world_id, X_WB, depth_camera, scene_graph))
    # Creating back cam
    X_WB = xyz_rpy_deg(
        [-2.2 / scaling + trans_x, 0 + trans_y, 0 + trans_z], [-90, 0, -90]
    )
    sensors.append(create_camera(builder, world_id, X_WB, depth_camera, scene_graph))

    if drake_visualizer:
        DrakeVisualizer.AddToBuilder(builder, scene_graph)

    plant.Finalize()
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()

    dofs = plant.num_actuated_dofs()
    if dofs != plant.num_positions():
        raise ValueError(
            "Error on converted model: Num positions is not equal to num actuated dofs."
        )

    if randomize_poses:
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
        sim_plant_context = plant.GetMyContextFromRoot(diagram_context)
        plant.get_actuation_input_port(model).FixValue(
            sim_plant_context, np.zeros((dofs, 1))
        )
        plant.SetPositions(sim_plant_context, model, joint_positions)

    for i, sensor in enumerate(sensors):
        generate_images_and_iou(
            sensor.GetMyMutableContextFromRoot(diagram_context),
            sensor,
            test_specific_temp_directory,
            pose_directory,
            i + 1,
            iou_test_threshold,
        )


def setup_temporary_model_description_file(
    model_directory, description_file, temp_directory, mesh_type
):
    # Setup model temporary files
    temp_test_model_path = os.path.join(temp_directory, mesh_type, "model")
    model_file_path = os.path.join(temp_test_model_path, description_file)
    shutil.copytree(model_directory, temp_test_model_path)
    root = etree.parse(model_file_path)
    model_name = root.find("model").attrib["name"]
    for uri in root.findall(".//uri"):
        uri.text = uri.text.replace("model://" + model_name + "/", "")

    # XML substitution workaround to visualize collisions this should be
    # doable through AssignRole and RemoveRole, but it's still needed for
    # gazebo anyway. It can be reformed once the gazebo model_photo_shoot
    # plugin adds support for collisions.
    if mesh_type == "collision":
        collision_tags = root.findall(".//collision")
        for visual_parent in root.findall(".//visual/.."):
            for visual_element in visual_parent.findall("visual"):
                visual_parent.remove(visual_element)
        for collision_tag in collision_tags:
            collision_tag.tag = "visual"
        # Remove tags that cause problems after the collision-visual swap
        tags = ["surface", "contact"]
        for tag in tags:
            for element_parent in root.findall(f".//{tag}/.."):
                for element in element_parent.findall(tag):
                    element_parent.remove(element)

    data = etree.tostring(root, pretty_print=True).decode("utf-8")
    with open(model_file_path, "w") as text_file:
        text_file.write(data)

    return model_file_path


def run_test(
    model_file_path,
    temp_directory,
    mesh_type,
    type_joint_positions,
    randomize_poses,
    render_camera_core,
    iou_test_threshold,
    drake_visualizer,
    poses_filename="poses.txt",
):
    # Setup temporary pics and metadata directory
    temp_default_pics_path = os.path.join(
        temp_directory, mesh_type, "pics", type_joint_positions
    )
    os.makedirs(temp_default_pics_path)
    current_dir = os.getcwd()
    os.chdir(temp_default_pics_path)
    plugin_config_path = os.path.join(temp_default_pics_path, "plugin_config.sdf")
    generate_sdf(
        model_file_path,
        os.path.join(temp_default_pics_path, poses_filename),
        randomize_poses,
        plugin_config_path,
        render_camera_core,
    )

    subprocess.run(
        f"ign gazebo -s -r --headless-rendering {plugin_config_path} --iterations 50",
        shell=True,
        check=True,
    )
    perform_iou_testing(
        model_file_path,
        os.path.join(temp_directory, mesh_type),
        type_joint_positions,
        randomize_poses,
        render_camera_core,
        iou_test_threshold,
        drake_visualizer,
    )
    os.chdir(current_dir)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--model_directory",
        required=True,
        help="Directory location of the model files",
    )
    parser.add_argument(
        "-m", "--model_description_file", help="Model description file name", default=""
    )
    parser.add_argument(
        "-t",
        "--temp_directory",
        required=True,
        help="Temporary directory file where temporary objects will be written",
    )
    parser.add_argument(
        "-i",
        "--iou_threshold",
        help="Threshold for the intesrsection over union test",
        type=float,
        default=0.9,
    )
    parser.add_argument("-v", "--drake_visualizer", action="store_true")
    args = parser.parse_args()

    render_camera_core = RenderCameraCore(
        "renderer",
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
    )

    mesh_type = "visual"
    tmp_model_file_path = setup_temporary_model_description_file(
        args.model_directory,
        args.model_description_file,
        args.temp_directory,
        mesh_type,
    )
    print("Running default pose, visual mesh test:")
    run_test(
        tmp_model_file_path,
        args.temp_directory,
        mesh_type,
        "default_pose",
        False,
        render_camera_core,
        args.iou_threshold,
        args.drake_visualizer,
    )
    print("Running random pose, visual mesh test:")
    run_test(
        tmp_model_file_path,
        args.temp_directory,
        mesh_type,
        "random_pose",
        True,
        render_camera_core,
        args.iou_threshold,
        args.drake_visualizer,
    )

    print("Running default pose, collision mesh test:")
    mesh_type = "collision"
    tmp_model_file_path = setup_temporary_model_description_file(
        args.model_directory,
        args.model_description_file,
        args.temp_directory,
        mesh_type,
    )
    run_test(
        tmp_model_file_path,
        args.temp_directory,
        mesh_type,
        "default_pose",
        False,
        render_camera_core,
        args.iou_threshold,
        args.drake_visualizer,
    )
    print("Running random pose, collision mesh test:")
    run_test(
        tmp_model_file_path,
        args.temp_directory,
        mesh_type,
        "random_pose",
        True,
        render_camera_core,
        args.iou_threshold,
        args.drake_visualizer,
    )


if __name__ == "__main__":
    try:
        main()
        print()
        print("[ Done ]")
    except UserError as e:
        eprint(e)
        sys.exit(1)
