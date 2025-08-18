import os
from subprocess import run

from python.runfiles import runfiles


def main():
    try:
        from lib.ros_environment.unique import (
            enforce_unique_ros_environment
        )
        enforce_unique_ros_environment()
    except ImportError:
        pass
    r = runfiles.Create()
    cc_bin = r.Rlocation(
        "drake_ros_examples/examples/rs_flip_flop/rs_flip_flop"
    )
    py_bin = r.Rlocation(
        "drake_ros_examples/examples/rs_flip_flop/rs_flip_flop_py"
    )
    run([cc_bin, "--simulation_sec=0.01"], env=env, check=True)
    run([py_bin, "--simulation_sec=0.01"], env=env, check=True)
    print("[ Done ]")


if __name__ == '__main__':
    main()
