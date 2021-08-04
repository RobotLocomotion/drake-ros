# -*- python -*-

load(
    "@REPOSITORY_ROOT@:distro.bzl",
    "RUNTIME_ENVIRONMENT"
)
load(
    "@REPOSITORY_ROOT@:common.bzl",
    "incorporate_rmw_implementation",
    "incorporate_fastrtps_profile"
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
    "generate_isolated_fastrtps_profile",
)


def ros_cc_binary(name, rmw_implementation = None, **kwargs):
    """
    Builds a C/C++ binary, injecting the runtime environment
    specific to this ROS overlay.
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
        if "fastrtps" in rmw_implementation:
            if "block-network" in kwargs.get("tags", []):
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                binary_kwargs, binary_env_changes = \
                    incorporate_fastrtps_profile(
                        binary_kwargs, binary_env_changes, profile_name
                    )

    native.cc_binary(
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
    native.cc_binary(name = name, **kwargs)
    
def ros_cc_test(name, rmw_implementation = None, **kwargs):
    """
    Builds a C/C++ test, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the cc_test() rule.
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
        if "fastrtps" in rmw_implementation:
            if "block-network" in kwargs.get("tags", []):
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                test_kwargs, test_env_changes = \
                    incorporate_fastrtps_profile(
                        test_kwargs, test_env_changes, profile_name
                    )

    native.cc_test(
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
    native.cc_test(name = name, **kwargs)
