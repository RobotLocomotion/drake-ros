import pytest
import random
import sys

import drake_ros
import drake_ros._cc_generic_typecaster as temp

from geometry_msgs.msg import Polygon, Point32, Quaternion

# This message type is not covered by geometry_messages_pybind.h
# and hence must be processed using the generic typecaster.
def test_msg_polygon():
    p = Polygon()
    for _ in range(10):
        point = Point32()
        point.x = float(random.randint(1, 100))
        point.y = float(random.randint(1, 100))
        point.z = float(random.randint(1, 100))
        p.points.append(point)
    print(p.points)
    msg_converted = temp.testTypecasting(msg=p)
    assert p == msg_converted

# This message is typecasted using geometry_messages_pybind.h
# but since the module is not included, this should still be
# processed using the generic typecaster.
def test_msg_quaternion():
    q = Quaternion()
    q.x = 1.0
    q.y = 2.0
    q.z = 3.0
    q.w = 4.0

    msg_converted = temp.testTypecasting(msg=q)
    assert q == msg_converted


if __name__ == '__main__':
    sys.exit(pytest.main(sys.argv))
