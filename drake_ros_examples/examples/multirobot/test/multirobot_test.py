import os
import subprocess

from rmw_isolation import isolate_rmw_by_path
from rules_python.python.runfiles import runfiles


def check_binary(binary):
    r = runfiles.Create()
    env = dict(os.environ)
    env.update(r.EnvVars())
    p = subprocess.check_call(
        [r.Rlocation(binary), "--simulation_sec", "0.01"],
        env=env)


def main():
    if "TEST_TMPDIR" in os.environ:
        isolate_rmw_by_path(os.environ["TEST_TMPDIR"])
        os.environ["ROS_HOME"] = os.path.join(os.environ["TEST_TMPDIR"])

    check_binary("drake_ros_examples/examples/multirobot/multirobot")
    check_binary("drake_ros_examples/examples/multirobot/multirobot_py")

if __name__ == '__main__':
    main()
