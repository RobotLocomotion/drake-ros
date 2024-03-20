"""
Roll up script for plotjuggler with ROS.
"""


import os
import subprocess
from bazel_tools.tools.python.runfiles import runfiles

def main():
    r = runfiles.Create()
    binary_path = r.Rlocation("ros2/plotjuggler_plotjuggler")

    if not binary_path:
        raise FileNotFoundError("Could not find my_binary")

    # Run the binary
    subprocess.run(binary_path, check=True)

if __name__ == "__main__":
    main()
