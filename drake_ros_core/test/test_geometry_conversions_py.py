import pytest
import pydrake.math
import _drake_ros_core
import numpy as np

from geometry_msgs.msg import Quaternion, Point

# TODO (aditya)
def test_translation():
    # ROS point to Vector3 (numpy array)
    p = Point()
    p.x = 1.12
    p.y = 2.34
    p.z = 3.456

    array_converted = _drake_ros_core.ros_point_to_vector3(p)
    array_expected = np.array([[1.12],[2.34],[3.456]])

    assert (array_expected == array_converted).all()

    # Vector3 (numpy array) to ROS Point
    # TODO

def test_orientation():
    # ROS quaternion to rotation matrix.
    q = Quaternion()
    q.x = 0.5
    q.y = 0.5
    q.z = 0.5
    q.w = 0.5

    rot_matrix_converted = \
        _drake_ros_core.ros_quaternion_to_rotation_matrix(q)

    rot_matrix_expected = \
        np.array([[0.0, 0.0, 1.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]])

    assert (rot_matrix_expected == rot_matrix_converted.matrix()).all()

    # Rotation matrix to ROS quaternion.
    quaternion_expected = \
            _drake_ros_core.rotation_matrix_to_ros_quaternion(rot_matrix_converted)
    assert (quaternion_expected == q)

# TODO (aditya)
def test_pose():
    # ROS pose to rigid transform.

    # Rigid transform to ROS pose.

    # ROS transform to rigid transform.

    # Rigid transform to ROS transform.
    pass

# TODO (aditya)
def test_spatial_velocity():
    pass

# TODO (aditya)
def test_spatial_acceleration():
    pass

# TODO (aditya)
def test_spatial_force():
    pass
