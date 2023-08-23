import subprocess

print("Hello world from roslaunch_base")

subprocess.run(["./external/ros2/ros2", "launch", "ros2_example_apps/eg_launch.py"])
# subprocess.run(["/bin/bash"])
