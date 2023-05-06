#!/usr/bin/env python3
#    topics:      /k4a/color/camera_info   : sensor_msgs/CameraInfo 
#                 /k4a/color/image         : sensor_msgs/Image      
#                 /k4a/depth/camera_info   : sensor_msgs/CameraInfo 
#                 /k4a/depth/image         : sensor_msgs/Image      
#                 /k4a/points              : sensor_msgs/PointCloud2
#                 /tf                      : tf2_msgs/TFMessage
#
# All topics are published at the same rate, about 30 Hz
# Total data rate about 16.9 GB per minute

import math
import time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf2_msgs.msg import TFMessage

import rclpy


def sec_to_stamp(time_sec, stamp):
    stamp.sec = int(time_sec)
    stamp.nanosec = int((time_sec - int(time_sec)) * 1e9)


def fake_tf_msg(time_sec):
    msg = TFMessage()

    # Frame 1
    transform_stamped = TransformStamped()
    sec_to_stamp(time_sec, transform_stamped.header.stamp)
    transform_stamped.header.frame_id = "k4a_frame"
    transform_stamped.child_frame_id = "k4a_color_optical_frame"
    transform_stamped.transform.translation.x = -0.03202302551269531
    transform_stamped.transform.translation.y = -0.0018641867637634278
    transform_stamped.transform.translation.z = 0.0038453204631805423
    transform_stamped.transform.rotation.x = -0.05098445576786709
    transform_stamped.transform.rotation.y = -0.0002824597694676041
    transform_stamped.transform.rotation.z = -0.003070799777573787
    transform_stamped.transform.rotation.x = 0.9986946859151996
    msg.transforms.append(transform_stamped)

    # Frame 2
    transform_stamped = TransformStamped()
    sec_to_stamp(time_sec, transform_stamped.header.stamp)
    transform_stamped.header.frame_id = "k4a_frame"
    transform_stamped.child_frame_id = "k4a_depth_optical_frame"
    transform_stamped.transform.translation.x = 0.0
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = 0.0
    transform_stamped.transform.rotation.x = 1.0

    msg.transforms.append(transform_stamped)
    return msg


def fake_color_camera_info(time_sec):
    msg = CameraInfo()
    sec_to_stamp(time_sec, msg.header.stamp)
    msg.header.frame_id = "k4a_color_optical_frame"
    msg.height = 720
    msg.width = 1280
    msg.distortion_model = "plumb_bob"
    msg.d = [0.48298484086990356, -2.89497971534729, 0.0008299206383526325, -0.0010119054932147264, 1.7135130167007446]
    msg.k = [608.921630859375, 0.0, 638.298095703125, 0.0, 609.2725219726562, 366.1363220214844, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [608.921630859375, 0.0, 638.298095703125, 0.0, 0.0, 609.2725219726562, 366.1363220214844, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.height = 0
    msg.roi.width = 0
    msg.roi.do_rectify = False
    return msg


def fake_depth_camera_info(time_sec):
    msg = CameraInfo()
    sec_to_stamp(time_sec, msg.header.stamp)
    msg.header.frame_id = "k4a_depth_optical_frame"
    msg.height = 576
    msg.width = 640
    msg.distortion_model = "plumb_bob"
    msg.d = [7.4479804039001465, 4.547260284423828, -4.0661790990270674e-05, 0.00011088199244113639, 0.20779651403427124]
    msg.k = [504.0411682128906, 0.0, 327.29541015625, 0.0, 504.5057678222656, 329.4179382324219, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [504.0411682128906, 0.0, 327.29541015625, 0.0, 0.0, 504.5057678222656, 329.4179382324219, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.height = 0
    msg.roi.width = 0
    msg.roi.do_rectify = False
    return msg


def fake_color_image(time_sec):
    msg = fake_color_image.msg
    if msg is None:
        fake_color_image.msg = Image()
        msg = fake_color_image.msg
        msg.header.frame_id = "k4a_color_optical_frame"
        msg.height = 720
        msg.width = 1280
        msg.encoding = "bgra8"
        msg.is_bigendian = 0
        msg.step = 5120
        msg.data = bytes([d % 255 for d in range(3686400)])
    sec_to_stamp(time_sec, msg.header.stamp)
    return msg

fake_color_image.msg = None


def fake_depth_image(time_sec):
    msg = fake_depth_image.msg
    if msg is None:
        fake_depth_image.msg = Image()
        msg = fake_depth_image.msg
        msg.header.frame_id = "k4a_depth_optical_frame"
        msg.height = 576
        msg.width = 640
        msg.encoding = "mono16"
        msg.is_bigendian = 0
        msg.step = 1280
        msg.data = bytes([d % 255 for d in range(737280)])
    sec_to_stamp(time_sec, msg.header.stamp)
    return msg

fake_depth_image.msg = None


def fake_points(time_sec):
    msg = fake_points.msg
    if msg is None:
        fake_points.msg = PointCloud2()
        msg = fake_points.msg
        msg.header.frame_id = "k4a_depth_optical_frame"
        msg.height = 576
        msg.width = 640
        msg.fields = []
        x_field = PointField()
        x_field.name = "x"
        x_field.offset = 0
        x_field.datatype = 7
        x_field.count = 1
        msg.fields.append(x_field)
        y_field = PointField()
        y_field.name = "y"
        y_field.offset = 4
        y_field.datatype = 7
        y_field.count = 1
        msg.fields.append(y_field)
        z_field = PointField()
        z_field.name = "z"
        z_field.offset = 8
        z_field.datatype = 7
        z_field.count = 1
        msg.fields.append(z_field)
        rgb_field = PointField()
        rgb_field.name = "rgb"
        rgb_field.offset = 12
        rgb_field.datatype = 7
        rgb_field.count = 1
        msg.fields.append(rgb_field)
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 10240
        msg.data = bytes([d % 255 for d in range(5898240)])
        msg.is_dense = True
    sec_to_stamp(time_sec, msg.header.stamp)
    return msg

fake_points.msg = None


def main():
    rclpy.init()
    node = rclpy.create_node('fake_k4a')
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    color_camera_info_pub = node.create_publisher(CameraInfo, '/k4a/color/camera_info', 1)
    depth_camera_info_pub = node.create_publisher(CameraInfo, '/k4a/depth/camera_info', 1)
    color_image_pub = node.create_publisher(Image, '/k4a/color/image', 1)
    depth_image_pub = node.create_publisher(Image, '/k4a/depth/image', 1)
    points_pub = node.create_publisher(PointCloud2, '/k4a/points', 1)

    rate = node.create_rate(30)
    while rclpy.ok():
        rclpy.spin_once(node)
        now = time.time()
        tf_pub.publish(fake_tf_msg(now))
        color_camera_info_pub.publish(fake_color_camera_info(now))
        depth_camera_info_pub.publish(fake_depth_camera_info(now))
        color_image_pub.publish(fake_color_image(now))
        depth_image_pub.publish(fake_depth_image(now))
        points_pub.publish(fake_points(now))
        rate.sleep()


if __name__ == '__main__':
    main()