import pytest
import pydrake.math
import _drake_ros_core

from geometry_msgs.msg import Quaternion

# TODO (aditya) -Add these when Eigen python bindings are resolved.
def test_translation():
    pass

def test_orientation():
    # ROS quaternion to rotation matrix.
    q1 = Quaternion()
    q1.x = 1.0
    q1.y = 1.0
    q1.z = 1.0
    q1.w = 1.0

    q2 = _drake_ros_core.ros_quaternion_to_rotation_matrix(q1)

    # TODO (aditya) - Add expectations for q2

    # Rotation matrix to ROS quaternion.
    q3 = pydrake.math.RotationMatrix_[float]()

    q4 = _drake_ros_core.rotation_matrix_to_ros_quaternion(q3)

    # TODO (aditya) - Add expectations for q4

def test_pose():
    # ROS pose to rigid transform.

    # Rigid transform to ROS pose.

    # ROS transform to rigid transform.

    # Rigid transform to ROS transform.
    pass

def test_spatial_velocity():
    pass

def test_spatial_acceleration():
    pass

def test_spatial_force():
    pass
