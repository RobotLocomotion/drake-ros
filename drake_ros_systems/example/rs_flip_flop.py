#!/usr/bin/env python3

from drake_ros_systems import RosInterfaceSystem
from drake_ros_systems import RosPublisherSystem
from drake_ros_systems import RosSubscriberSystem

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
# from pydrake.systems.lcm import LcmInterfaceSystem
# from pydrake.lcm import DrakeLcm
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem

from std_msgs.msg import String


def traced(func, ignoredirs=None):
    """Decorates func such that its execution is traced, but filters out any
     Python code outside of the system prefix."""
    import functools
    import sys
    import trace
    if ignoredirs is None:
        ignoredirs = ["/usr", sys.prefix]
    tracer = trace.Trace(trace=1, count=0, ignoredirs=ignoredirs)

    @functools.wraps(func)
    def wrapped(*args, **kwargs):
        return tracer.runfunc(func, *args, **kwargs)

    return wrapped


class HelloWorld(LeafSystem):
    """Outputs Hello World bool message."""

    def __init__(self):
        super().__init__()

        self.DeclareAbstractOutputPort(
            'text',
            lambda: AbstractValue.Make(String()),
            self._do_output_text)

    def _do_output_text(self, context, data):
        data.get_mutable_value().data = "hello world"


# @traced
def main():
    builder = DiagramBuilder()

    sys_ros_interface = builder.AddSystem(RosInterfaceSystem())

    # can add publisher systems
    # Need bindings for subscriber system
    # Need python version of memory class
    # Need python version of NAND gate class
    # Need boilerplate to create system

    # rps = RosPublisherSystem(String, "asdf", ris.get_ros_interface())

    sys_hello_world = builder.AddSystem(HelloWorld())
    sys_ros_pub = builder.AddSystem(
        RosPublisherSystem(String, "asdf", sys_ros_interface.get_ros_interface()))

    builder.Connect(
        sys_hello_world.get_output_port(0),
        sys_ros_pub.get_input_port(0)
    )

    sys_ros_sub_pt = builder.AddSystem(RosSubscriberSystem(String, "input", sys_ros_interface.get_ros_interface()));
    sys_ros_pub_pt = builder.AddSystem(RosPublisherSystem(String, "output", sys_ros_interface.get_ros_interface()));
    builder.Connect(
        sys_ros_sub_pt.get_output_port(0),
        sys_ros_pub_pt.get_input_port(0)
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    while True:
        simulator.AdvanceTo(simulator_context.get_time() + 0.1)


if __name__ == '__main__':
    main()
