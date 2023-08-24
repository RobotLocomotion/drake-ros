import subprocess
import sys
import os

def find(name, path):
    for root, _, files in os.walk(path):
        if name in files:
            return os.path.relpath(os.path.join(root, name))

launch_file_name = sys.argv[1]

roslaunch_cli = "./external/ros2/ros2"
action = "launch"
# TODO : Is there a better way to locate the launch file exactly ?
launch_file = find(launch_file_name, os.getcwd())

subprocess.run([roslaunch_cli, action, launch_file])
