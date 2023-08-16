import os
import math
import sys

import numpy as np

import drake_ros.core
from drake_ros.core import RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterSystem
from drake_ros.tf2 import SceneTfBroadcasterParams

from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.math import RollPitchYaw
from pydrake.math import RotationMatrix
from pydrake.geometry import FramePoseVector
from pydrake.geometry import GeometryFrame
from pydrake.geometry import SceneGraph

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import TriggerType
from pydrake.systems.primitives import ConstantValueSource

import pytest

import rclpy
import rclpy.time
import tf2_ros


def isolate_if_using_bazel():
    # Do not require `make_unique_ros_isolation_env` module for CMake.
    # TODO(eric.cousineau): Expose this to CMake in better location..
    try:
        from bazel_ros_env import make_unique_ros_isolation_env
        os.environ.update(make_unique_ros_isolation_env())
    except ImportError:
        assert "TEST_TMPDIR" not in os.environ


def test_nominal_case():
    drake_ros.core.init()

    builder = DiagramBuilder()

    sys_ros_interface = builder.AddSystem(
        RosInterfaceSystem('test_tf_broadcaster_py'))

    scene_graph = builder.AddSystem(SceneGraph())
    source_id = scene_graph.RegisterSource('test_source')
    odom_frame = scene_graph.RegisterFrame(
        source_id, GeometryFrame('odom'))
    base_link_frame = scene_graph.RegisterFrame(
        source_id, odom_frame, GeometryFrame('base_link'))

    X_WO = RigidTransform(
        R=RotationMatrix.Identity(),
        p=np.array([1., 1., 0.]))
    X_OB = RigidTransform(
        rpy=RollPitchYaw(0., 0., math.pi / 2.),
        p=np.array([1., 1., 0.]))

    pose_vector = FramePoseVector()
    pose_vector.set_value(odom_frame, X_WO)
    pose_vector.set_value(base_link_frame, X_OB)

    pose_vector_source = builder.AddSystem(
        ConstantValueSource(AbstractValue.Make(pose_vector)))

    builder.Connect(
        pose_vector_source.get_output_port(),
        scene_graph.get_source_pose_port(source_id))

    scene_tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            sys_ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kForced}
            )
        )
    )

    builder.Connect(
        scene_graph.get_query_output_port(),
        scene_tf_broadcaster.get_graph_query_input_port())

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

    rclpy.init()
    node = rclpy.create_node('tf_listener')

    buffer_ = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer_, node, spin_thread=False)  # noqa

    time = rclpy.time.Time(seconds=13.)
    stamp = time.to_msg()

    context.SetTime(time.nanoseconds / 1e9)
    diagram.ForcedPublish(context)

    future = buffer_.wait_for_transform_async('world', 'odom', time)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)

    assert future.done()
    assert future.result()
    world_to_odom = buffer_.lookup_transform('world', 'odom', time)
    assert world_to_odom.header.stamp.sec == stamp.sec
    assert world_to_odom.header.stamp.nanosec == stamp.nanosec
    p_WO = X_WO.translation()
    assert math.isclose(world_to_odom.transform.translation.x, p_WO[0])
    assert math.isclose(world_to_odom.transform.translation.y, p_WO[1])
    assert math.isclose(world_to_odom.transform.translation.z, p_WO[2])
    R_WO = X_WO.rotation().ToQuaternion()
    assert math.isclose(world_to_odom.transform.rotation.x, R_WO.x())
    assert math.isclose(world_to_odom.transform.rotation.y, R_WO.y())
    assert math.isclose(world_to_odom.transform.rotation.z, R_WO.z())
    assert math.isclose(world_to_odom.transform.rotation.w, R_WO.w())

    assert buffer_.can_transform('odom', 'base_link', time)
    odom_to_base_link = buffer_.lookup_transform('odom', 'base_link', time)
    assert odom_to_base_link.header.stamp.sec == stamp.sec
    assert odom_to_base_link.header.stamp.nanosec == stamp.nanosec
    p_OB = X_OB.translation()
    assert math.isclose(odom_to_base_link.transform.translation.x, p_OB[0])
    assert math.isclose(odom_to_base_link.transform.translation.y, p_OB[1])
    assert math.isclose(odom_to_base_link.transform.translation.z, p_OB[2])
    R_OB = X_OB.rotation().ToQuaternion()
    assert math.isclose(odom_to_base_link.transform.rotation.x, R_OB.x())
    assert math.isclose(odom_to_base_link.transform.rotation.y, R_OB.y())
    assert math.isclose(odom_to_base_link.transform.rotation.z, R_OB.z())
    assert math.isclose(odom_to_base_link.transform.rotation.w, R_OB.w())


if __name__ == '__main__':
    isolate_if_using_bazel()
    sys.exit(pytest.main(sys.argv))
