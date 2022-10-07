"""Python wrapper for drake_ros_core."""

from _drake_ros_core import add_clock_publisher
from _drake_ros_core import init
from _drake_ros_core import RosInterfaceSystem
from _drake_ros_core import RosPublisherSystem
from _drake_ros_core import RosSubscriberSystem
from _drake_ros_core import SerializerInterface
from _drake_ros_core import shutdown

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import TriggerType

from rclpy.serialization import serialize_message
from rclpy.serialization import deserialize_message
from rclpy.type_support import check_for_type_support
import rclpy.qos

import rosgraph_msgs.msg


class PySerializer(SerializerInterface):
    """
    A (de)serialization interface for Python ROS messages.
    """

    def __init__(self, message_type):
        SerializerInterface.__init__(self)
        check_for_type_support(message_type)
        self._message_type = message_type

    def GetTypeSupport(self):
        return self._message_type._TYPE_SUPPORT

    def CreateDefaultValue(self):
        return AbstractValue.Make(self._message_type())

    def Serialize(self, abstract_value):
        return serialize_message(abstract_value.get_value())

    def Deserialize(self, serialized_message, abstract_value):
        abstract_value.set_value(deserialize_message(
            serialized_message, self._message_type))


@staticmethod
def _make_ros_publisher_system(
    message_type, topic_name, qos, ros_interface,
    publish_triggers={
        TriggerType.kPerStep,
        TriggerType.kForced},
    publish_period=0.0
):
    return RosPublisherSystem(
        PySerializer(message_type),
        topic_name, qos, ros_interface,
        publish_triggers, publish_period)


RosPublisherSystem.Make = _make_ros_publisher_system


@staticmethod
def _make_ros_subscriber_system(
    message_type, topic_name, qos, ros_interface
):
    return RosSubscriberSystem(
        PySerializer(message_type),
        topic_name, qos, ros_interface)


RosSubscriberSystem.Make = _make_ros_subscriber_system


# def add_clock_publisher(
#     builder, ros_interface,
#     topic_name="/clock",
#     qos=None,
#     publish_triggers={
#         TriggerType.kPerStep,
#         TriggerType.kForced},
#     publish_period=0.0
# ):
#     if qos is None:
#         qos = rclpy.qos.QoSProfile(
#             depth=1,
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
# 
#     clock_sys = builder.AddSystem(ClockSystem())
# 
#     pub_sys = builder.AddSystem(RosPublisherSystem.Make(
#         rosgraph_msgs.msg.Clock, topic_name, qos, ros_interface,
#         publish_triggers, publish_period))
# 
#     builder.Connect(
#         clock_sys.get_output_port(),
#         pub_sys.get_input_port());


__all__ = [
    'add_clock_publisher',
    'ClockSystem',
    'DrakeRosInterface',
    'init',
    'PySerializer',
    'RosInterfaceSystem',
    'RosPublisherSystem',
    'RosSubscriberSystem',
    'SerializerInterface',
    'shutdown',
]
