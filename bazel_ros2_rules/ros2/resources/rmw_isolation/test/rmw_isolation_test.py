import argparse
import os
from multiprocessing import Process
import sys
import tempfile
from time import sleep

import rclpy
import rclpy.node
import std_msgs
from std_msgs.msg import Float64

class Talker(rclpy.node.Node):
    def __init__(self, id):
        super().__init__('talker_' + str(id))
        self._publisher = self.create_publisher(
                std_msgs.msg.Float64, 'chatter', 10)
        self._timer = self.create_timer(0.1, self._timer_callback)
        self._id = id

    def _timer_callback(self):
        msg = std_msgs.msg.Float64()
        msg.data = float(self._id)
        self._publisher.publish(msg)

class Listener(rclpy.node.Node):
    def __init__(self, id):
        super().__init__('listener_' + str(id))
        self._subscription = self.create_subscription(
            std_msgs.msg.Float64,
            'chatter',
            self._topic_callback,
            10)
        timeout = self.declare_parameter('timeout', 1.0)
        self._timer = self.create_timer(
            timeout.value, self._timer_callback)
        self._expected_messages_received = 0
        self._id = id

    def _topic_callback(self, msg):
        assert msg.data == self._id, \
            f"I heard '{msg.data}' yet I was expecting '{self._id}'!"
        self._expected_messages_received += 1

    def _timer_callback(self):
        assert self._expected_messages_received > 0, \
            f"I did not hear '{self._id}' even once!"
        rclpy.shutdown()

# Launch a process for the talker or the listener.
def launch_node(id, node_type="talker"):
    # Create an isolated enviroment.
    log_directory = tempfile.TemporaryDirectory()
    os.environ["ROS_LOG_DIR"] = log_directory.name

    directory_path = os.path.join(os.getcwd(), str(id))
    if not os.path.exists(directory_path): os.mkdir(directory_path)

    from rmw_isolation import isolate_rmw_by_path
    isolate_rmw_by_path(directory_path)

    # Start the talker.
    rclpy.init()
    try:
        if node_type == "talker":
            rclpy.spin(Talker(id))
            main()
        elif node_type == "listener":
            rclpy.spin(Listener(id))
    except rclpy.executors.ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        rclpy.shutdown()
    except Exception:
        rclpy.shutdown()
        raise

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--number_of_isolated_pairs", type=int, default=5)
    args = parser.parse_args()

    # Declare processes.
    talker_processes = [Process(target=launch_node, args=(i, "talker"))
                        for i in range(args.number_of_isolated_pairs)]
    listener_processes = [Process(target=launch_node, args=(i, "listener"))
                          for i in range(args.number_of_isolated_pairs)]

    # Start the processes.
    for talker in talker_processes:
        talker.start()

    # Wait for the talkers to start.
    sleep(1.0)

    for listener in listener_processes:
        listener.start()

    sleep(2)

    for talker, listener in zip(talker_processes, listener_processes):
        talker.kill()
        listener.kill()

if __name__ == "__main__":
    main()
