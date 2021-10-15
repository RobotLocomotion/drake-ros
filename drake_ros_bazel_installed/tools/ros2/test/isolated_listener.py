import os

import rclpy
import rclpy.node
import rclpy.executors
import std_msgs.msg


class IsolatedListener(rclpy.node.Node):

    def __init__(self):
        super().__init__('isolated_listener')
        self._subscription = self.create_subscription(
            std_msgs.msg.String,
            'topic',
            self._topic_callback,
            10)
        timeout = self.declare_parameter('timeout', 2.0)
        self._timer = self.create_timer(
            timeout.value, self._timer_callback)

    def _topic_callback(self, msg):
        assert False, f"I heard '{msg.data}'!"

    def _timer_callback(self):
        rclpy.shutdown()

if __name__ == '__main__':
    if 'TEST_TMPDIR' in os.environ:
        from drake_ros.tools.ros2.rmw_isolation import isolate_rmw_by_path
        isolate_rmw_by_path(os.environ['TEST_TMPDIR'])

    rclpy.init()

    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        rclpy.spin(IsolatedListener(), executor)
    finally:
        # NOTE(hidmic): try_shutdown raises AttributeError
        # Need https://github.com/ros2/rclpy/pull/812
        pass  # rclpy.try_shutdown()
