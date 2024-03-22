import pytest
import random
import sys

import drake_ros
from drake_ros.ros_message_type_caster_test_via_specific_types import (
    TestTypecastingViaSpecificMacro,
)
from drake_ros.ros_message_type_caster_test_via_all import (
    TestTypecastingViaAllMacro,
)

from geometry_msgs.msg import Polygon, Point32, Quaternion


def make_quaternion():
    q = Quaternion()
    q.x = 1.0
    q.y = 2.0
    q.z = 3.0
    q.w = 4.0
    return q


def make_polygon():
    p = Polygon()
    for _ in range(10):
        point = Point32()
        point.x = float(random.randint(1, 100))
        point.y = float(random.randint(1, 100))
        point.z = float(random.randint(1, 100))
        p.points.append(point)
    return p


def test_msg_quaternion():
    """
    This should convert via ROS_MSG_PYBIND_TYPECAST_ALL() and via
    ROS_MSG_PYBIND_TYPECAST()
    """
    q = make_quaternion()

    msg_converted = TestTypecastingViaAllMacro(msg=q)
    assert q == msg_converted

    msg_converted = TestTypecastingViaSpecificMacro(msg=q)
    assert q == msg_converted


def test_msg_polygon():
    """
    This should convert via ROS_MSG_PYBIND_TYPECAST_ALL(), but *not*
    via ROS_MSG_PYBIND_TYPECAST() because we explicitly did not
    declare it so.
    """
    p = make_polygon()

    msg_converted = TestTypecastingViaAllMacro(msg=p)
    assert p == msg_converted

    with pytest.raises(TypeError):
        TestTypecastingViaSpecificMacro(msg=p)


if __name__ == '__main__':
    sys.exit(pytest.main(sys.argv))
