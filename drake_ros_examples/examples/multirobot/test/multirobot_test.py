import subprocess

from bazel_tools.python.runfiles import runfiles


def main():
    try:
        from lib.ros_environment.unique import (
            enforce_unique_ros_environment
        )
        enforce_unique_ros_environment()
    except ImportError:
        pass
    r = runfiles.Create()
    cc_bin = r.Rlocation("drake_ros_examples/examples/multirobot/multirobot")
    py_bin = r.Rlocation("drake_ros_examples/examples/multirobot/multirobot_py")
    subprocess.run([cc_bin, "--simulation_sec=0.01"], check=True)
    subprocess.run([py_bin, "--simulation_sec=0.01"], check=True)
    print("[ Done ]")


if __name__ == '__main__':
    main()
