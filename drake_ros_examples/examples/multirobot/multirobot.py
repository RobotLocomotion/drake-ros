#!/usr/bin/env python3

import argparse
import os
import os.path
import pathlib

import numpy

import drake_ros.core
from drake_ros.core import ClockSystem
from drake_ros.core import RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterSystem
from drake_ros.tf2 import SceneTfBroadcasterParams

from drake_ros.viz import RvizVisualizer
from drake_ros.viz import RvizVisualizerParams

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import DrakeVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.multibody.tree import JointIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import TriggerType
from pydrake.systems.primitives import ConstantVectorSource


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--simulation_sec',
        type=float,
        default=float('inf'),
        help='How many seconds to run the simulation')
    args = parser.parse_args()

    # Create a Drake diagram
    builder = DiagramBuilder()
    # Initialise the ROS infrastructure
    drake_ros.core.init()
    # Create a Drake system to interface with ROS
    sys_ros_interface = builder.AddSystem(RosInterfaceSystem('multirobot'))
    ClockSystem.AddToBuilder(builder, sys_ros_interface.get_ros_interface())

    # Add a multibody plant and a scene graph to hold the robots
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001, discrete_contact_solver="sap"),
        builder,
    )

    viz_dt = 1 / 32.0
    # Add a TF2 broadcaster to provide task frame information
    scene_tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            sys_ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_dt,
            )
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_tf_broadcaster.get_graph_query_input_port())

    # Add a system to output the visualisation markers for rviz
    scene_visualizer = builder.AddSystem(
        RvizVisualizer(
            sys_ros_interface.get_ros_interface(),
            params=RvizVisualizerParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_dt,
            )
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_visualizer.get_graph_query_input_port())

    # Prepare to load the robot model
    parser = Parser(plant)
    model_file_path = FindResourceOrThrow(
        'drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf')
    model_name = "kuka_iiwa"

    # Create a 5x5 array of manipulators
    NUM_ROWS = 5
    NUM_COLS = 5
    models = []
    for x in range(NUM_ROWS):
        models.append([])
        for y in range(NUM_COLS):
            # Load the model from the file and give it a name based on its X
            # and Y coordinates in the array
            models[x].append(parser.AddModelFromFile(
                model_file_path,
                model_name + str(x) + '_' + str(y)))

            # Weld the robot to world so it doesn't fall through floor
            base_frame = plant.GetFrameByName("base", models[x][y])
            X_WB = RigidTransform([x, y, 0])
            plant.WeldFrames(plant.world_frame(), base_frame, X_WB)

    # Finalise the multibody plant to make it ready for use
    plant.Finalize()

    # Set the control input of each robot to uncontrolled
    for x in range(NUM_ROWS):
        for y in range(NUM_COLS):
            # Get the number of degrees of freedom for the robot
            nu = plant.num_actuated_dofs(models[x][y])
            # Create a vector with the same number of zeros
            u0 = numpy.zeros(nu)
            # Create a system that emits a constant value using that vector
            constant = builder.AddSystem(ConstantVectorSource(u0))
            # Connect the constant value to the robot
            builder.Connect(
                constant.get_output_port(0),
                plant.get_actuation_input_port(models[x][y]))

    # Add a Drake visualiser instance to the diagram
    viz = DrakeVisualizer.AddToBuilder(builder, scene_graph)

    # Build the complete system from the diagram
    diagram = builder.Build()

    # Create a simulator for the system
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    # Step the simulator in 0.1s intervals
    step = 0.1
    while simulator_context.get_time() < args.simulation_sec:
        next_time = min(
            simulator_context.get_time() + step, args.simulation_sec,
        )
        simulator.AdvanceTo(next_time)


if __name__ == '__main__':
    main()
