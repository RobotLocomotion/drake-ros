import pytest
import pydrake.math
import pydrake.multibody.math
import pydrake.common.eigen_geometry
import _drake_ros_core
import numpy as np

from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Accel, Wrench, Pose, Transform

def test_ros_point_to_vector3():
    p = Point()
    p.x = 1.12
    p.y = 2.34
    p.z = 3.456
    array_converted = _drake_ros_core.ros_point_to_vector3(p)
    array_expected = np.array([[1.12],[2.34],[3.456]])

    assert (array_expected == array_converted).all()

def test_vector3_to_ros_point():
    p = Point()
    p.x = 1.12
    p.y = 2.34
    p.z = 3.456
    point_expected = \
            _drake_ros_core.vector3_to_ros_point(np.array([[1.12],[2.34],[3.456]]))
    assert point_expected == p

def test_ros_vector3_to_vector3():
    v = Vector3()
    v.x = 1.25
    v.y = 2.50
    v.z = 3.75
    vec3_converted = _drake_ros_core.ros_vector3_to_vector3(v)
    vec3_expected = np.array([[1.25], [2.50], [3.75]])
    assert (vec3_converted == vec3_expected).all()

def test_vector3_to_ros_vector3():
    v = Vector3()
    v.x = 1.25
    v.y = 2.50
    v.z = 3.75
    ros_vec3_expected = \
            _drake_ros_core.vector3_to_ros_vector3(np.array([[1.25], [2.50], [3.75]]))
    assert ros_vec3_expected == v

def test_ros_quaternion_to_quaternion():
    q = Quaternion()
    q.x = 0.1
    q.y = 0.2
    q.z = 0.3
    q.w = 0.4
    pydrake_quaternon_expected = _drake_ros_core.ros_quaternion_to_quaternion(q)
    assert pydrake_quaternon_expected.x() == 0.1
    assert pydrake_quaternon_expected.y() == 0.2
    assert pydrake_quaternon_expected.z() == 0.3
    assert pydrake_quaternon_expected.w() == 0.4

def test_quaternion_to_ros_quaternion():
    ros_quaternion = _drake_ros_core.quaternion_to_ros_quaternion(
            pydrake.common.eigen_geometry.Quaternion([1/np.sqrt(30), 2/np.sqrt(30),
                                                      3/np.sqrt(30), 4/np.sqrt(30)]))
    assert ros_quaternion.w == 1/np.sqrt(30)
    assert ros_quaternion.x == 2/np.sqrt(30)
    assert ros_quaternion.y == 3/np.sqrt(30)
    assert ros_quaternion.z == 4/np.sqrt(30)

def test_ros_quaternion_to_rotation_matrix():
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

def test_rotation_matrix_to_ros_quaternion():
    q = Quaternion()
    q.x = 0.5
    q.y = 0.5
    q.z = 0.5
    q.w = 0.5
    rot_matrix = pydrake.math.RotationMatrix(
        np.array([[0.0, 0.0, 1.0],
                  [1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]]))
    quaternion_expected = \
            _drake_ros_core.rotation_matrix_to_ros_quaternion(rot_matrix)
    assert quaternion_expected == q

def test_ros_pose_to_rigid_transform():
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

    assert np.allclose(rigid_transform_expected.rotation().matrix(),
            np.array([[-0.66666667, 0.13333333 , 0.73333333],
                      [0.66666667, -0.33333333, 0.6666667],
                      [0.33333333, 0.93333333, 0.13333333]]))

def test_rigid_transform_to_ros_pose():
    rigid_transform = pydrake.math.RigidTransform(
            p = np.array([1.0, 2.0, 3.0]),
            quaternion = pydrake.common.eigen_geometry.Quaternion(
                [1.0/np.sqrt(30), 2.0/np.sqrt(30), 3.0/np.sqrt(30), 4.0/np.sqrt(30)]))

    ros_pose_expected = _drake_ros_core.rigid_transform_to_ros_pose(rigid_transform)
    assert ros_pose_expected.position.x == 1.0
    assert ros_pose_expected.position.y == 2.0
    assert ros_pose_expected.position.z == 3.0
    assert np.isclose(ros_pose_expected.orientation.w, 1.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.x, 2.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.y, 3.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.z, 4.0/np.sqrt(30))

def test_ros_transform_to_rigid_transform():
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
    assert np.allclose(rigid_transform_expected.rotation().matrix(),
            np.array([[-0.66666667, 0.13333333 , 0.73333333],
                      [0.66666667, -0.33333333, 0.6666667],
                      [0.33333333, 0.93333333, 0.13333333]]))

def test_rigid_transform_to_ros_transform():
    rigid_transform = pydrake.math.RigidTransform(
            p = np.array([1.0, 2.0, 3.0]),
            quaternion = pydrake.common.eigen_geometry.Quaternion(
                [1.0/np.sqrt(30), 2.0/np.sqrt(30), 3.0/np.sqrt(30), 4.0/np.sqrt(30)]))
    ros_transform_expected = _drake_ros_core.rigid_transform_to_ros_transform(rigid_transform)
    assert ros_transform_expected.translation.x == 1.0
    assert ros_transform_expected.translation.y == 2.0
    assert ros_transform_expected.translation.z == 3.0
    assert np.isclose(ros_transform_expected.rotation.w, 1.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.x, 2.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.y, 3.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.z, 4.0/np.sqrt(30))

def test_ros_pose_to_isometry3():
    ros_pose = Pose()
    ros_pose.position.x = 1.0
    ros_pose.position.y = 2.0
    ros_pose.position.z = 3.0
    ros_pose.orientation.w = 1.0
    ros_pose.orientation.x = 2.0
    ros_pose.orientation.y = 3.0
    ros_pose.orientation.z = 4.0
    isometry_expected = _drake_ros_core.ros_pose_to_isometry3(ros_pose)
    assert (isometry_expected.translation() == np.array([1.0, 2.0, 3.0])).all()
    assert np.isclose(isometry_expected.quaternion().w(), 1.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().x(), 2.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().y(), 3.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().z(), 4.0/np.sqrt(30))

def test_isometry3_to_ros_pose():
    isometry3 = pydrake.common.eigen_geometry.Isometry3(
            translation = np.array([1.0, 2.0, 3.0]),
            rotation = np.array([[-0.66666667, 0.13333333 , 0.73333333],
                                 [0.66666667, -0.33333333, 0.6666667],
                                 [0.33333333, 0.93333333, 0.13333333]]))

    ros_pose_expected = _drake_ros_core.isometry3_to_ros_pose(isometry3)
    assert ros_pose_expected.position.x == 1.0
    assert ros_pose_expected.position.y == 2.0
    assert ros_pose_expected.position.z == 3.0
    assert np.isclose(ros_pose_expected.orientation.w, 1.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.x, 2.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.y, 3.0/np.sqrt(30))
    assert np.isclose(ros_pose_expected.orientation.z, 4.0/np.sqrt(30))

def test_ros_transform_to_isometry3():
    ros_transform = Transform()
    ros_transform.translation.x = 1.0
    ros_transform.translation.y = 2.0
    ros_transform.translation.z = 3.0
    ros_transform.rotation.w = 1.0
    ros_transform.rotation.x = 2.0
    ros_transform.rotation.y = 3.0
    ros_transform.rotation.z = 4.0
    isometry_expected = _drake_ros_core.ros_transform_to_isometry3(ros_transform)
    assert (isometry_expected.translation() == np.array([1.0, 2.0, 3.0])).all()
    assert np.isclose(isometry_expected.quaternion().w(), 1.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().x(), 2.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().y(), 3.0/np.sqrt(30))
    assert np.isclose(isometry_expected.quaternion().z(), 4.0/np.sqrt(30))

def test_isometry3_to_ros_transform():
    isometry3 = pydrake.common.eigen_geometry.Isometry3(
            translation = np.array([1.0, 2.0, 3.0]),
            rotation = np.array([[-0.66666667, 0.13333333 , 0.73333333],
                                 [0.66666667, -0.33333333, 0.6666667],
                                 [0.33333333, 0.93333333, 0.13333333]]))
    ros_transform_expected = _drake_ros_core.isometry3_to_ros_transform(isometry3)
    assert ros_transform_expected.translation.x == 1.0
    assert ros_transform_expected.translation.y == 2.0
    assert ros_transform_expected.translation.z == 3.0
    assert np.isclose(ros_transform_expected.rotation.w, 1.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.x, 2.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.y, 3.0/np.sqrt(30))
    assert np.isclose(ros_transform_expected.rotation.z, 4.0/np.sqrt(30))

def test_spatial_velocity_to_ros_twist():
    t = Twist()
    t.linear.x = 1.11
    t.linear.y = 2.22
    t.linear.z = 3.33
    t.angular.x = 11.11
    t.angular.y = 22.22
    t.angular.z = 33.33
    ros_twist_converted = _drake_ros_core.spatial_velocity_to_ros_twist(
            pydrake.multibody.math.SpatialVelocity_[float](
                w = np.array([11.11, 22.22, 33.33]),
                v = np.array([1.11, 2.22, 3.33])
                )
            )
    assert ros_twist_converted == t

def test_ros_twist_to_spatial_velocity():
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

def test_ros_twist_to_vector6():
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

def test_vector6_to_ros_twist():
    t = Twist()
    t.linear.x = 1.11
    t.linear.y = 2.22
    t.linear.z = 3.33
    t.angular.x = 11.11
    t.angular.y = 22.22
    t.angular.z = 33.33
    ros_twist_expected = _drake_ros_core.vector6_to_ros_twist(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert ros_twist_expected == t

def test_ros_accel_to_vector6():
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

def vector6_to_ros_accel():
    a = Accel()
    a.linear.x = 1.11
    a.linear.y = 2.22
    a.linear.z = 3.33
    a.angular.x = 11.11
    a.angular.y = 22.22
    a.angular.z = 33.33
    ros_accel_expected = _drake_ros_core.vector6_to_ros_accel(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert ros_accel_expected == a

def test_ros_accel_to_spatial_acceleration():
    a = Accel()
    a.linear.x = 1.11
    a.linear.y = 2.22
    a.linear.z = 3.33
    a.angular.x = 11.11
    a.angular.y = 22.22
    a.angular.z = 33.33
    spatial_accel_converted = _drake_ros_core.ros_accel_to_spatial_acceleration(a)
    assert (np.array([1.11, 2.22, 3.33]) ==
            spatial_accel_converted.translational()).all()
    assert (np.array([11.11, 22.22, 33.33]) ==
            spatial_accel_converted.rotational()).all()

def test_spatial_acceleration_to_ros_accel():
    a = Accel()
    a.linear.x = 1.11
    a.linear.y = 2.22
    a.linear.z = 3.33
    a.angular.x = 11.11
    a.angular.y = 22.22
    a.angular.z = 33.33
    ros_accel_converted = _drake_ros_core.spatial_acceleration_to_ros_accel(
            pydrake.multibody.math.SpatialAcceleration_[float](
                alpha = np.array([11.11, 22.22, 33.33]),
                a = np.array([1.11, 2.22, 3.33])
                )
            )
    assert ros_accel_converted == a

def test_ros_wrench_to_vector6():
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

def test_vector6_to_ros_wrench():
    w = Wrench()
    w.force.x = 1.11
    w.force.y = 2.22
    w.force.z = 3.33
    w.torque.x = 11.11
    w.torque.y = 22.22
    w.torque.z = 33.33
    ros_wrench_expected = _drake_ros_core.vector6_to_ros_wrench(
            np.array([[11.11], [22.22], [33.33], [1.11], [2.22], [3.33]]))
    assert ros_wrench_expected == w

def test_ros_wrench_to_spatial_force():
    w = Wrench()
    w.force.x = 1.11
    w.force.y = 2.22
    w.force.z = 3.33
    w.torque.x = 11.11
    w.torque.y = 22.22
    w.torque.z = 33.33
    spatial_force_converted = _drake_ros_core.ros_wrench_to_spatial_force(w)
    assert (np.array([1.11, 2.22, 3.33]) ==
            spatial_force_converted.translational()).all()
    assert (np.array([11.11, 22.22, 33.33]) ==
            spatial_force_converted.rotational()).all()

def test_spatial_force_to_ros_wrench():
    w = Wrench()
    w.force.x = 1.11
    w.force.y = 2.22
    w.force.z = 3.33
    w.torque.x = 11.11
    w.torque.y = 22.22
    w.torque.z = 33.33
    ros_wrench_converted = _drake_ros_core.spatial_force_to_ros_wrench(
            pydrake.multibody.math.SpatialForce_[float](
                tau = np.array([11.11, 22.22, 33.33]),
                f = np.array([1.11, 2.22, 3.33])
                )
            )
    assert ros_wrench_converted == w
