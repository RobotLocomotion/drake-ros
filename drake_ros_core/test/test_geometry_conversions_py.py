import pytest
import pydrake.math
import pydrake.multibody.math
import pydrake.common.eigen_geometry
import _drake_ros_core
import numpy as np

from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Accel, Wrench, Pose, Transform

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
    # ROS quaternion to pydrake.common.eigen_geometry.Quaternion
    q = Quaternion()
    q.x = 0.1
    q.y = 0.2
    q.z = 0.3
    q.w = 0.4

    pydrake_quaternon_expected = _drake_ros_core.ros_quaternion_to_quaternion(q)
    assert (pydrake_quaternon_expected.x() == 0.1)
    assert (pydrake_quaternon_expected.y() == 0.2)
    assert (pydrake_quaternon_expected.z() == 0.3)
    assert (pydrake_quaternon_expected.w() == 0.4)

    # pydrake.common.eigen_geometry.Quaternion to ROS quaternion
    ros_quaternion = _drake_ros_core.quaternion_to_ros_quaternion(
            pydrake.common.eigen_geometry.Quaternion([1/np.sqrt(30), 2/np.sqrt(30),
                                                      3/np.sqrt(30), 4/np.sqrt(30)]))
    assert (ros_quaternion.w == 1/np.sqrt(30))
    assert (ros_quaternion.x == 2/np.sqrt(30))
    assert (ros_quaternion.y == 3/np.sqrt(30))
    assert (ros_quaternion.z == 4/np.sqrt(30))

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
    ros_pose = Pose()
    ros_pose.position.x = 1.0
    ros_pose.position.y = 2.0
    ros_pose.position.z = 3.0

    ros_pose.orientation.w = 1.0
    ros_pose.orientation.x = 2.0
    ros_pose.orientation.y = 3.0
    ros_pose.orientation.z = 4.0

    rigid_transform_expected = _drake_ros_core.ros_pose_to_rigid_transform(ros_pose)
    assert (rigid_transform_expected.translation() ==
            np.array([1.0, 2.0, 3.0])).all()

    assert (np.allclose(rigid_transform_expected.rotation().matrix(),
            np.array([[-0.66666667, 0.13333333 , 0.73333333],
                      [0.66666667, -0.33333333, 0.6666667],
                      [0.33333333, 0.93333333, 0.13333333]])))

    # Rigid transform to ROS pose.

    # ROS transform to rigid transform.
    ros_transform = Transform()
    ros_transform.translation.x = 1.0
    ros_transform.translation.y = 2.0
    ros_transform.translation.z = 3.0

    ros_transform.rotation.w = 1.0
    ros_transform.rotation.x = 2.0
    ros_transform.rotation.y = 3.0
    ros_transform.rotation.z = 4.0

    rigid_transform_expected = _drake_ros_core.ros_transform_to_rigid_transform(ros_transform)
    assert (rigid_transform_expected.translation() ==
            np.array([1.0, 2.0, 3.0])).all()

    assert (np.allclose(rigid_transform_expected.rotation().matrix(),
            np.array([[-0.66666667, 0.13333333 , 0.73333333],
                      [0.66666667, -0.33333333, 0.6666667],
                      [0.33333333, 0.93333333, 0.13333333]])))

    # Rigid transform to ROS transform.

    # ROS Pose to Isometry3

    # Isometry3 to ROS Pose

def test_spatial_velocity():
    # ROS Twist to Vec6
    t = Twist()
    t.linear.x = 1.11
    t.linear.y = 2.22
    t.linear.z = 3.33

    t.angular.x = 11.11
    t.angular.y = 22.22
    t.angular.z = 33.33

    vec6_expected = _drake_ros_core.ros_twist_to_vector6(t)
    assert (vec6_expected == np.array([[11.11], [22.22], [33.33],
                                       [1.11],[2.22],[3.33]])).all()

    # Vec6 to ROS Twist
    ros_twist_expected = _drake_ros_core.vector6_to_ros_twist(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert (ros_twist_expected == t)

    # ROS Twist to Spatial Velocity
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

def test_spatial_acceleration():
    # ROS Accel to Vec6
    a = Accel()
    a.linear.x = 1.11
    a.linear.y = 2.22
    a.linear.z = 3.33

    a.angular.x = 11.11
    a.angular.y = 22.22
    a.angular.z = 33.33

    vec6_expected = _drake_ros_core.ros_accel_to_vector6(a)
    assert (vec6_expected == np.array([[11.11], [22.22], [33.33],
                                       [1.11],[2.22],[3.33]])).all()
    # Vec6 to ROS Accel
    ros_accel_expected = _drake_ros_core.vector6_to_ros_accel(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert (ros_accel_expected == a)

    # ROS Accel to Spatial Acceleration
    spatial_accel_converted = _drake_ros_core.ros_accel_to_spatial_acceleration(a)
    assert (np.array([1.11, 2.22, 3.33]) ==
            spatial_accel_converted.translational()).all()
    assert (np.array([11.11, 22.22, 33.33]) ==
            spatial_accel_converted.rotational()).all()

    # Spatial Acceleration to ROS Accel
    ros_accel_converted = _drake_ros_core.spatial_acceleration_to_ros_accel(
            pydrake.multibody.math.SpatialAcceleration_[float](
                alpha = np.array([11.11, 22.22, 33.33]),
                a = np.array([1.11, 2.22, 3.33])
                )
            )
    assert (ros_accel_converted == a)

def test_spatial_force():
    # ROS Wrench to Vec6
    w = Wrench()
    w.force.x = 1.11
    w.force.y = 2.22
    w.force.z = 3.33

    w.torque.x = 11.11
    w.torque.y = 22.22
    w.torque.z = 33.33

    vec6_expected = _drake_ros_core.ros_wrench_to_vector6(w)
    assert (vec6_expected == np.array([[11.11], [22.22], [33.33],
                                       [1.11],[2.22],[3.33]])).all()
    # Vec6 to ROS Wrench
    ros_wrench_expected = _drake_ros_core.vector6_to_ros_wrench(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert (ros_wrench_expected == w)

    # ROS Wrench to Spatial Force
    spatial_force_converted = _drake_ros_core.ros_wrench_to_spatial_force(w)
    assert (np.array([1.11, 2.22, 3.33]) ==
            spatial_force_converted.translational()).all()
    assert (np.array([11.11, 22.22, 33.33]) ==
            spatial_force_converted.rotational()).all()

    # Spatial Force to ROS Wrench
    ros_wrench_converted = _drake_ros_core.spatial_force_to_ros_wrench(
            pydrake.multibody.math.SpatialForce_[float](
                tau = np.array([11.11, 22.22, 33.33]),
                f = np.array([1.11, 2.22, 3.33])
                )
            )
    assert (ros_wrench_converted == w)
