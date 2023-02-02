import pytest
import pydrake.math
import pydrake.multibody.math
import _drake_ros_core
import numpy as np

from geometry_msgs.msg import Quaternion, Point, Vector3, Twist

def test_translation():
    # ROS Point to Vector3 (numpy array)
    p = Point()
    p.x = 1.12
    p.y = 2.34
    p.z = 3.456

    array_converted = _drake_ros_core.ros_point_to_vector3(p)
    array_expected = np.array([[1.12],[2.34],[3.456]])

    assert (array_expected == array_converted).all()

    # Vector3 (numpy array) to ROS Point
    point_expected = \
            _drake_ros_core.vector3_to_ros_point(np.array([[1.12],[2.34],[3.456]]))
    assert (point_expected == p)

    # ROS Vector3 to Vector3 (numpy array)
    v = Vector3()
    v.x = 1.25
    v.y = 2.50
    v.z = 3.75

    vec3_converted = _drake_ros_core.ros_vector3_to_vector3(v)
    vec3_expected = np.array([[1.25], [2.50], [3.75]])
    assert (vec3_converted == vec3_expected).all()

    # Vector3 (numpy array) to ROS Vector3
    ros_vec3_expected = \
            _drake_ros_core.vector3_to_ros_vector3(np.array([[1.25], [2.50], [3.75]]))
    assert (ros_vec3_expected == v)

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
    # ROS Twist to Spatial Velocity
    t = Twist()
    t.linear.x = 1.11
    t.linear.y = 2.22
    t.linear.z = 3.33

    t.angular.x = 11.11
    t.angular.y = 22.22
    t.angular.z = 33.33

    spatial_vel_converted = _drake_ros_core.ros_twist_to_spatial_velocity(t)
    assert (np.array([1.11, 2.22, 3.33]) ==
            spatial_vel_converted.translational()).all()
    assert (np.array([11.11, 22.22, 33.33]) ==
            spatial_vel_converted.rotational()).all()

    # Spatial Velocity to ROS Twist
    ros_twist_converted = _drake_ros_core.spatial_velocity_to_ros_twist(
            pydrake.multibody.math.SpatialVelocity_[float](
                w = np.array([11.11, 22.22, 33.33]),
                v = np.array([1.11, 2.22, 3.33])
                )
            )
    assert (ros_twist_converted == t)

# TODO (aditya)
def test_spatial_acceleration():
    pass

# TODO (aditya)
def test_spatial_force():
    pass
