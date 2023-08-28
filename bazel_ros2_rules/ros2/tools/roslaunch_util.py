import subprocess
import sys
import os

from launch.actions import ExecuteProcess

def find_rel_path(name, path):
    for root, _, files in os.walk(path):
        if name in files:
            return os.path.relpath(os.path.join(root, name))

# Launch action to wrap over ExecuteProcess.
def ExecuteBazelTarget(bazel_target_name):
    target_path = find_rel_path(bazel_target_name, os.getcwd())
    return ExecuteProcess(
            cmd = [target_path]
        )

if __name__ == '__main__':
    launch_file_name = sys.argv[1]

    roslaunch_cli = "./external/ros2/ros2"
    action = "launch"
    # TODO : Is there a better way to locate the launch file exactly ?
    launch_file = find_rel_path(launch_file_name, os.getcwd())

    env = {**os.environ, 'PYTHONPATH': os.getcwd() + '/external/bazel_ros2_rules/ros2/tools:'
           + os.environ['PYTHONPATH']}
    subprocess.run([roslaunch_cli, action, launch_file], env=env)
    # TODO (Aditya): For debugging, to be removed
    # subprocess.run(["/bin/bash"], env=env)
