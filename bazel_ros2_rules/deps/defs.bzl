load("//deps/cpython:defs.bzl", "cpython_local_repository")

def bazel_ros2_rules_dependencies(omit_python_dev=False):
    if not omit_python_dev:
        cpython_local_repository(name = "python_dev")
