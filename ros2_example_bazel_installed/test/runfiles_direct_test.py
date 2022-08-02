"""
Simulates running tests outside of `bazel run` and `bazel test`.
In effect, this isolates the environment to the bare minimum, ensuring the test
does not have access to env vars like `RUNFILES*` or `TEST_TMPDIR`.
"""

import os
import subprocess

from bazel_tools.tools.python.runfiles import runfiles


def main():
    manifest = runfiles.Create()
    py_test = manifest.Rlocation("ros2_example_bazel_installed/runfiles_py_test")
    cc_test = manifest.Rlocation("ros2_example_bazel_installed/runfiles_cc_test")

    keys = {"HOME", "PWD", "USER", "PATH", "SHELL"}
    minimal_env = {}
    for key in keys:
        minimal_env[key] = os.environ[key]

    # Works
    subprocess.run([py_test], env=minimal_env, check=True)
    # Fails
    subprocess.run([cc_test], env=minimal_env, check=True)


assert __name__ == "__main__"
main()
