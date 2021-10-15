import rclpy
import rclpy.node
import std_msgs.msg


class Talker(rclpy.node.Node):

    def __init__(self):
        super().__init__('talker')
        self._publisher = self.create_publisher(
            std_msgs.msg.String, 'topic', 10)
        self._timer = self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        msg = std_msgs.msg.String()
        msg.data = 'Howdy'
        self._publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()

    try:
        rclpy.spin(Talker())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
