# -*- python -*-

load(
    "@REPOSITORY_ROOT@:distro.bzl",
    "RUNTIME_ENVIRONMENT"
)
load(
    "@REPOSITORY_ROOT@:common.bzl",
    "incorporate_rmw_implementation",
)
load(
    "@drake_ros//tools/skylark:dload_py.bzl",
    "dload_py_shim"
)
load(
    "@drake_ros//tools/skylark:kwargs.bzl",
    "keep_common",
    "remove_test_specific"
)

def ros_py_import(
    name, executable, rmw_implementation = None, py_binary_rule = native.py_binary, **kwargs
):
    """
    Imports an existing, potentially pre-built executable by wrapping it with a shim that
    will inject the minimal runtime environment necessary for execution when depending on
    this ROS 2 local repository.

    Akin to the cc_import() rule.

    Args:
        name: imported executable target name
        executable: path to an executable file
        rmw_implementation: optional RMW implementation to run against
        py_binary_rule: optional py_binary() rule override

    Additional keyword arguments are forwarded to the `py_binary_rule`.
    """

    env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        kwargs, env_changes = \
            incorporate_rmw_implementation(
                kwargs, env_changes,
                rmw_implementation = rmw_implementation
            )

    shim_name = name + "_shim.py"
    shim_kwargs = keep_common(kwargs)
    dload_py_shim(
        name = shim_name,
        target = executable,
        env_changes = env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        tags = ["nolint"] + kwargs.get("tags", []),
        data = [executable] + kwargs.get("data", []),
        deps = kwargs.get("deps", []) + [
            "@bazel_tools//tools/python/runfiles"]
    )
    py_binary_rule(name = name, **kwargs)

def ros_py_binary(
    name, rmw_implementation = None, py_binary_rule = native.py_binary, **kwargs
):
    """
    Builds a Python binary and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the py_binary() rule, which this rule decorates.

    Args:
        name: Python binary target name
        rmw_implementation: optional RMW implementation to run against
        py_binary_rule: optional py_binary() rule override

    Additional keyword arguments are forwarded to the `py_binary_rule`.
    """

    binary_name = "_" + name
    binary_kwargs = dict(kwargs)
    if "main" not in binary_kwargs:
        binary_kwargs["main"] = name + ".py"
    binary_env_changes = dict(RUNTIME_ENVIRONMENT)

    if rmw_implementation:
        binary_kwargs, binary_env_changes = \
            incorporate_rmw_implementation(
                binary_kwargs, binary_env_changes,
                rmw_implementation = rmw_implementation
            )

    py_binary_rule(
        name = binary_name,
        **binary_kwargs
    )

    shim_name = binary_name + "_shim.py"
    shim_kwargs = keep_common(kwargs)
    dload_py_shim(
        name = shim_name,
        target = ":" + binary_name,
        env_changes = binary_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        data = [":" + binary_name],
        deps = [
            "@bazel_tools//tools/python/runfiles",
            ":" + binary_name,  # Support py_binary being used a dependency
        ],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    py_binary_rule(name = name, **kwargs)

def ros_py_test(
    name, rmw_implementation = None,
    py_binary_rule = native.py_binary,
    py_test_rule = native.py_test, **kwargs
):
    """
    Builds a Python test and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the py_test() rule, which this rule decorates.

    Args:
        name: Python test target name
        rmw_implementation: optional RMW implementation to run against
        py_binary_rule: optional py_binary() rule override
        py_test_rule: optional py_test() rule override

    Additional keyword arguments are forwarded to the `py_test_rule` and to the
    `py_binary_rule` (minus the test specific ones).
    """
    binary_name = "_" + name
    binary_kwargs = remove_test_specific(kwargs)
    binary_kwargs.update(testonly = True)
    binary_env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        binary_kwargs, binary_env_changes = \
            incorporate_rmw_implementation(
                binary_kwargs, binary_env_changes,
                rmw_implementation = rmw_implementation,
            )

    py_binary_rule(
        name = binary_name,
        **binary_kwargs
    )

    shim_name = binary_name + "_shim.py"
    shim_kwargs = keep_common(kwargs)
    shim_kwargs.update(testonly = True)
    dload_py_shim(
        name = shim_name,
        target = ":" + binary_name,
        env_changes = binary_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        data = [":" + binary_name],
        deps = ["@bazel_tools//tools/python/runfiles"],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    py_test_rule(name = name, **kwargs)
