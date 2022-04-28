# -*- python -*-

load(
    "//tools:dload_cc.bzl",
    "dload_cc_shim"
)
load(
    "//tools:kwargs.bzl",
    "filter_to_only_common_kwargs",
    "remove_test_specific_kwargs"
)
load(
    "//tools:common.bzl",
    "incorporate_rmw_implementation",
)
load(
    ":distro.bzl",
    "RUNTIME_ENVIRONMENT"
)

def ros_cc_binary(
    name, rmw_implementation = None, cc_binary_rule = native.cc_binary, **kwargs
):
    """
    Builds a C/C++ binary and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the cc_binary() rule, which this rule decorates.

    Args:
        name: C/C++ binary target name
        rmw_implementation: optional RMW implementation to run against
        cc_binary_rule: optional cc_binary() rule override

    Additional keyword arguments are forwarded to the `cc_binary_rule`.
    """
    if kwargs.get("linkshared", False):
        native.cc_binary(name = name, **kwargs)
        return

    binary_name = "_" + name + "_shimmed"
    binary_kwargs = kwargs
    binary_env_changes = dict(RUNTIME_ENVIRONMENT)

    if rmw_implementation:
        binary_kwargs, binary_env_changes = \
            incorporate_rmw_implementation(
                binary_kwargs, binary_env_changes,
                rmw_implementation = rmw_implementation
            )

    cc_binary_rule(
        name = binary_name,
        **binary_kwargs
    )

    shim_name = "_" + name + "_shim.cc"
    shim_kwargs = filter_to_only_common_kwargs(kwargs)
    dload_cc_shim(
        name = shim_name,
        target = ":" + binary_name,
        env_changes = binary_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        data = [":" + binary_name],
        deps = ["@bazel_tools//tools/cpp/runfiles"],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    cc_binary_rule(name = name, **kwargs)

def ros_cc_test(
    name, rmw_implementation = None,
    cc_binary_rule = native.cc_binary,
    cc_test_rule = native.cc_test, **kwargs
):
    """
    Builds a C/C++ test and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the cc_test() rule.

    Args:
        name: C/C++ test target name
        rmw_implementation: optional RMW implementation to run against
        cc_binary_rule: optional cc_binary() rule override.
        cc_test_rule: optional cc_test() rule override

    Additional keyword arguments are forwarded to the `cc_test_rule` and to the
    `cc_binary_rule` (minus the test specific ones).
    """
    binary_name = "_" + name + "_shimmed"
    binary_kwargs = remove_test_specific_kwargs(kwargs)
    binary_kwargs.update(testonly = True)
    binary_env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        binary_kwargs, test_env_changes = \
            incorporate_rmw_implementation(
                binary_kwargs, binary_env_changes,
                rmw_implementation = rmw_implementation
            )

    cc_binary_rule(
        name = binary_name,
        **binary_kwargs
    )

    shim_name = "_" + name + "_shim.cc"
    shim_kwargs = filter_to_only_common_kwargs(kwargs)
    shim_kwargs.update(testonly = True)
    dload_cc_shim(
        name = shim_name,
        target = ":" + binary_name,
        env_changes = binary_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        data = [":" + binary_name],
        deps = ["@bazel_tools//tools/cpp/runfiles"],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    cc_test_rule(name = name, **kwargs)
