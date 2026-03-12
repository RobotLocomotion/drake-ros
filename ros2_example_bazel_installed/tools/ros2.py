"""
Bazel-based entry point for `ros2` binary, e.g.
    ros2 interfaces list
    ros2 topic list
    ros2 bag record
"""

import os
import sys

from python.runfiles import runfiles as runfiles_api


def main():
    runfiles = runfiles_api.Create()
    bin_file = runfiles.Rlocation("ros2/ros2")
    argv = [bin_file] + sys.argv[1:]
    os.execv(bin_file, argv)


assert __name__ == "__main__"
main()
