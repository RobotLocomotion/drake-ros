# -*- python -*-

load(
    "@REPOSITORY_ROOT@:distro.bzl",
    "RUNTIME_ENVIRONMENT"
)
load(
    "@REPOSITORY_ROOT@:common.bzl",
    "incorporate_cyclonedds_profile",
    "incorporate_fastrtps_profile",
    "incorporate_rmw_implementation",
)
load(
    "@drake_ros//tools/skylark:dload_cc.bzl",
    "dload_cc_shim"
)
load(
    "@drake_ros//tools/skylark:kwargs.bzl",
    "keep_common"
)
load(
    "@drake_ros//tools/skylark/ros2:rmw.bzl",
    "generate_isolated_cyclonedds_profile",
    "generate_isolated_fastrtps_profile",
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

    binary_name = "_" + name
    binary_kwargs = kwargs
    binary_env_changes = dict(RUNTIME_ENVIRONMENT)

    if rmw_implementation:
        binary_kwargs, binary_env_changes = \
            incorporate_rmw_implementation(
                binary_kwargs, binary_env_changes,
                rmw_implementation = rmw_implementation
            )
        if "block-network" in kwargs.get("tags", []):
            if "fastrtps" in rmw_implementation:
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                binary_kwargs, binary_env_changes = \
                    incorporate_fastrtps_profile(
                        binary_kwargs, binary_env_changes, profile_name
                    )
            if "cyclonedds" in rmw_implementation:
                profile_name = name + ".cyclonedds_profile.xml"
                generate_isolated_cyclonedds_profile(profile_name)
                binary_kwargs, binary_env_changes = \
                    incorporate_cyclonedds_profile(
                        binary_kwargs, binary_env_changes, profile_name
                    )

    cc_binary_rule(
        name = binary_name,
        **binary_kwargs
    )

    shim_name = binary_name + "_shim.cc"
    shim_kwargs = keep_common(kwargs)
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
    name, rmw_implementation = None, cc_test_rule = native.cc_test, **kwargs
):
    """
    Builds a C/C++ test and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the cc_test() rule, which this rule decorates.

    Args:
        name: C/C++ test target name
        rmw_implementation: optional RMW implementation to run against
        cc_test_rule: optional cc_test() rule override

    Additional keyword arguments are forwarded to the `cc_test_rule`.
    """
    test_name = "_" + name
    test_env_changes = dict(RUNTIME_ENVIRONMENT)
    kwargs["tags"] = ["manual"] + kwargs.get("tags", [])
    if rmw_implementation:
        test_kwargs, test_env_changes = \
            incorporate_rmw_implementation(
                test_kwargs, test_env_changes,
                rmw_implementation = rmw_implementation
            )
        if "block-network" in kwargs.get("tags", []):
            if "fastrtps" in rmw_implementation:
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                test_kwargs, test_env_changes = \
                    incorporate_fastrtps_profile(
                        test_kwargs, test_env_changes, profile_name
                    )
            if "cyclonedds" in rmw_implementation:
                profile_name = name + ".cyclonedds_profile.xml"
                generate_isolated_cyclonedds_profile(profile_name)
                test_kwargs, test_env_changes = \
                    incorporate_cyclonedds_profile(
                        test_kwargs, test_env_changes, profile_name
                    )

    cc_test_rule(
        name = test_name,
        **test_kwargs
    )

    shim_name = test_name + "_shim.cc"
    shim_kwargs = keep_common(kwargs)
    shim_kwargs.update(testonly = True)
    dload_cc_shim(
        name = shim_name,
        target = ":" + test_name,
        env_changes = test_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        data = [":" + test_name],
        deps = ["@bazel_tools//tools/cpp/runfiles"],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    cc_test_rule(name = name, **kwargs)
