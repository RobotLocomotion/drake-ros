import subprocess
import sys

print("ARGS: ", sys.argv[1:])
launch_file_name = sys.argv[1]
launch_file_dir = "ros2_example_apps"

roslaunch_cli = "./external/ros2/ros2"
action = "launch"
launch_file = launch_file_dir + "/" + launch_file_name

subprocess.run([roslaunch_cli, action, launch_file])
# subprocess.run(["/bin/bash"])
