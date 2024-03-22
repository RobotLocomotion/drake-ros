"""
Bazel-based entry point for `ros2` binary, e.g.
    ros2 interfaces list
    ros2 topic list
    ros2 bag record
"""

import os
import sys

from bazel_tools.tools.python.runfiles import runfiles


def main():
    manifest = runfiles.Create()
    bin_file = manifest.Rlocation("ros2/ros2")
    argv = [bin_file] + sys.argv[1:]
    os.execv(bin_file, argv)


assert __name__ == "__main__"
main()
