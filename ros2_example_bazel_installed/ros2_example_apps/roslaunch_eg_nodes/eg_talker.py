import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, *, max_count=10):
        super().__init__("node")
        qos = rclpy.qos.QoSProfile(
            depth=max_count,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(String, "topic", qos)
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


if __name__ == "__main__":
    main()
