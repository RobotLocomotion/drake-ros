import copy
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
import numpy as np
import rclpy
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw


def _write_pose_msg(X_AB, p, q):
    X_AB = RigidTransform(X_AB)
    p.x, p.y, p.z = X_AB.translation()
    q.w, q.x, q.y, q.z = X_AB.rotation().ToQuaternion().wxyz()


def to_ros_pose(X_AB):
    """Converts Drake transform to ROS pose."""
    msg = Pose()
    _write_pose_msg(X_AB, p=msg.position, q=msg.orientation)
    return msg


def to_ros_transform(X_AB):
    """Converts Drake transform to ROS transform."""
    msg = Transform()
    _write_pose_msg(X_AB, p=msg.translation, q=msg.rotation)
    return msg


def xyz_rpy(xyz, rpy):
    """Shorthand to create an isometry from XYZ and RPY."""
    return RigidTransform(R=RotationMatrix(rpy=RollPitchYaw(rpy)), p=xyz)


def xyz_rpy_deg(xyz, rpy_deg):
    return xyz_rpy(xyz, np.deg2rad(rpy_deg))


def frame_markers(radius=0.01, L=0.1, *, base_pose=RigidTransform(), header=None):
    """
    Creates frame markers (e.g. for showing a pose) given `radius` and
    `length`.
    """
    tpl = Marker()
    if header is not None:
        tpl.header = header
    tpl.type = Marker.CYLINDER
    tpl.scale.x = radius
    tpl.scale.y = radius
    tpl.scale.z = L
    tpl.color.a = 1.0

    msg_x = copy.deepcopy(tpl)
    msg_x.color.r = 1.0
    X_MAx = base_pose @ xyz_rpy_deg([L/2., 0., 0.], [0., -90., 0.])
    msg_x.pose = to_ros_pose(X_MAx)

    msg_y = copy.deepcopy(tpl)
    msg_y.color.g = 1.0
    X_MAy = base_pose @ xyz_rpy_deg([0., L/2., 0.], [90., 0., 0.])
    msg_y.pose = to_ros_pose(X_MAy)

    msg_z = copy.deepcopy(tpl)
    msg_z.color.b = 1.0
    X_MAz = base_pose @ xyz_rpy_deg([0., 0., L/2.], [0., 0., 0.])
    msg_z.pose = to_ros_pose(X_MAz)
    return [msg_x, msg_y, msg_z]


def make_silly_pose(t):
    T = 5.0
    w = 2 * np.pi / T
    th = w * t
    c = np.cos(th)
    s = np.sin(th)
    r = 0.1
    yaw_deg_max = 45
    return xyz_rpy_deg(
        xyz=[r * c, r * s, 0],
        rpy_deg=[0, 0, yaw_deg_max * s],
    )


def main():
    rclpy.init()

    node = rclpy.create_node("rviz_pub")
    pose_pub = node.create_publisher(PoseStamped, "via_pose", 10)
    marker_pub = node.create_publisher(MarkerArray, "via_marker_array", 10)
    tf_broadcast = TransformBroadcaster(node)

    time_point_zero = Time()

    t_start = time.time()
    while True:
        # Create a silly pose.
        t_rel = time.time() - t_start
        X_WM = make_silly_pose(t_rel)

        # Offset poses so we can see visual difference.
        dz = 0.15
        X_WM_pose = RigidTransform([0, 0, 0]) @ X_WM
        X_WM_marker = RigidTransform([0, 0, dz]) @ X_WM
        X_WM_tf = RigidTransform([0, 0, 2 * dz]) @ X_WM

        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        # header.stamp = time_point_zero
        header.frame_id = "world"

        # GOOD: This works OK.
        message = PoseStamped()
        message.header = header
        message.pose = to_ros_pose(X_WM_pose)
        pose_pub.publish(message)

        # BAD: This only visualizes the 
        message = MarkerArray()
        message.markers = frame_markers(
            base_pose=X_WM_marker,
            header=header,
        )
        marker_pub.publish(message)

        # BAD: If using current time: extrapolation error about message being
        # ~1 - 5ms old :(
        # BAD: If using time_point_zero: error about TF_OLD_DATA :(
        message = TransformStamped()
        message.header = header
        message.child_frame_id = "via_tf"
        message.transform = to_ros_transform(X_WM_tf)
        tf_broadcast.sendTransform(message)

        # rclpy.spin_once(node)  # this causes things to hang?


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
