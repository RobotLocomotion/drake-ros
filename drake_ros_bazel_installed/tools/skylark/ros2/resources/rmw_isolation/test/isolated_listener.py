import os

import rclpy
import rclpy.node
import rclpy.executors
import std_msgs.msg


class IsolatedListener(rclpy.node.Node):

    def __init__(self, uuid):
        super().__init__('isolated_listener')
        self._subscription = self.create_subscription(
            std_msgs.msg.String,
            'uuid',
            self._topic_callback,
            10)
        timeout = self.declare_parameter('timeout', 2.0)
        self._timer = self.create_timer(
            timeout.value, self._timer_callback)
        self._expected_messages_received = 0
        self._uuid = uuid

    def _topic_callback(self, msg):
        assert msg.data == self._uuid, \
            f"I heard '{msg.data}' yet I was expecting '{self._uuid}'!"
        self._expected_messages_received += 1

    def _timer_callback(self):
        assert self._expected_messages_received > 0, \
            f"I did not hear '{elf._uuid}' even once!"
        rclpy.shutdown()


def main():
    if 'TEST_TMPDIR' in os.environ:
        from rmw_isolation import isolate_rmw_by_path
        isolate_rmw_by_path(os.environ['TEST_TMPDIR'])

    rclpy.init()
    uuid = os.environ.get('TEST_TMPDIR', 'none')
    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        rclpy.spin(IsolatedListener(uuid), executor)
    finally:
        # NOTE(hidmic): try_shutdown raises AttributeError
        # Need https://github.com/ros2/rclpy/pull/812
        pass  # rclpy.try_shutdown()


if __name__ == '__main__':
    main()
