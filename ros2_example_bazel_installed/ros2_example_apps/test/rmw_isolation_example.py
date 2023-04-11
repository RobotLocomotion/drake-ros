import os
from multiprocessing import Process
import sys
import tempfile
from time import sleep

import rclpy
import rclpy.node
import std_msgs
from std_msgs.msg import Float64

# This simple example demonstrates how a publisher and a subscriber
# can be isolated using rmw_isoaltion. We need to create a new temporary directory,
# and supply it to isolate_rmw_by_path() before invoking rclpy.init().
# This isolation works process wide, and internally it creates a config file for the rmw layer.
# For a complete test, refer to :
# bazel_ros2_rules/ros2/resources/rmw_isolation/test/rmw_isolation_test.py

class Talker(rclpy.node.Node):
    def __init__(self):
        super().__init__('talker_example')
        self._publisher = self.create_publisher(
                std_msgs.msg.Float64, 'chatter', 10)
        self._timer = self.create_timer(0.1, self._timer_callback)

    def _timer_callback(self):
        msg = std_msgs.msg.Float64()
        msg.data = 1.0
        self._publisher.publish(msg)

class Listener(rclpy.node.Node):
    def __init__(self):
        super().__init__('listener_example')
        self._subscription = self.create_subscription(
            std_msgs.msg.Float64,
            'chatter',
            self._topic_callback,
            10)
        self._expected_messages_received = 0

    def _topic_callback(self, _):
        self._expected_messages_received += 1

# Launch a process for the talker or the listener.
def launch_node(directory_path, node_type="talker"):
    # Create a logging directory for ROS.
    log_directory = tempfile.TemporaryDirectory()
    os.environ["ROS_LOG_DIR"] = log_directory.name

    # RMW isolation is implemented using a dedicated path
    # for the config file. Invoking isolate_rmw_by_path()
    # before rclpy.init() isolates this process and the nodes in it.
    from rmw_isolation import isolate_rmw_by_path
    isolate_rmw_by_path(directory_path)

    # Start the ROS node.
    rclpy.init()
    try:
        if node_type == "talker":
            rclpy.spin(Talker("test_node_pair"))
            main()
        elif node_type == "listener":
            rclpy.spin(Listener("test_node_pair"))
    except rclpy.executors.ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        rclpy.shutdown()
    except Exception:
        rclpy.shutdown()
        raise

def main():
    # Generate a temporary directory which will hold the config file for RMW isoaltion.
    directory_path = os.path.join(os.getcwd(), "test_node_pair")
    if not os.path.exists(directory_path): os.mkdir(directory_path)

    # The talker and listener processes are supplied a common directory_path
    # for rmw isoaltion, and hence will be able to talk to each other but will be isolated
    # from rest of the system. For e.g, if one were to run a new subscriber on the /chatter topic,
    # the data published by the talker would not be visible.
    talker_process = Process(target=launch_node, args=(directory_path, "talker"))
    listener_process = Process(target=launch_node, args=(directory_path, "listener"))

    # Wait for the talker to start.
    talker_process.start()
    sleep(1.0)

    listener_process.start()

    sleep(2)

    talker_process.kill()
    listener_process.kill()

if __name__ == "__main__":
    main()
