import os
from subprocess import run

from bazel_ros_env import (
    Rlocation,
    make_bazel_runfiles_env,
    make_unique_ros_isolation_env,
)


def make_env():
    env = dict(os.environ)
    env.update(make_bazel_runfiles_env())
    env.update(make_unique_ros_isolation_env())
    return env


def main():
    env = make_env()
    cc_bin = Rlocation("drake_ros_examples/examples/hydroelastic/hydroelastic")
    run([cc_bin, "--simulation_sec=0.01"], env=env, check=True)
    print("[ Done ]")


if __name__ == '__main__':
    main()
