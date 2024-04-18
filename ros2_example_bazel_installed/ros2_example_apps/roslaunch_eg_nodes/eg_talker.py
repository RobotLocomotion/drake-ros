import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, *, max_count = 10):
        super().__init__("node")
        self.publisher = self.create_publisher(String, "topic", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.max_count = max_count

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.count}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.count += 1

    def is_done(self):
        return self.count >= self.max_count


def main():
    rclpy.init()

    node = MinimalPublisher()

    # Use explicit `spin_once` so we can manually check for completion.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok() and not node.is_done():
        executor.spin_once(timeout_sec=1e-3)

    # Avoid odd error:
    #   cannot use Destroyable because destruction was requested
    executor._sigint_gc = None


if __name__ == "__main__":
    main()
