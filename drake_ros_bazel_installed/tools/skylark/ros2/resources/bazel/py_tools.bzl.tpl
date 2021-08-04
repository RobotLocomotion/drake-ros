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
    "@drake_ros//tools/skylark:dload_py.bzl",
    "dload_py_shim"
)
load(
    "@drake_ros//tools/skylark:kwargs.bzl",
    "keep_common"
)
load(
    "@drake_ros//tools/skylark/ros2:rmw.bzl",
    "generate_isolated_fastrtps_profile",
)

def ros_py_import(name, executable, rmw_implementation = None, **kwargs):
    """
    Imports an executable, injecting the runtime environment
    specific to this ROS overlay.

    Args:
        executable: executable file

    Akin to the cc_import() rule.
    """

    env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        kwargs, env_changes = \
            incorporate_rmw_implementation(
                kwargs, env_changes,
                rmw_implementation = rmw_implementation
            )
        if "fastrtps" in rmw_implementation:
            if "block-network" in kwargs.get("tags", []):
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                kwargs, env_changes = \
                    incorporate_fastrtps_profile(
                        kwargs, env_changes, profile_name
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
    native.py_binary(name = name, **kwargs)

def ros_py_binary(name, rmw_implementation = None, **kwargs):
    """
    Builds a Python binary, injecting the runtime environment
    specific to this ROS overlay.
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
        if "fastrtps" in rmw_implementation:
            if "block-network" in binary_kwargs.get("tags", []):
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                binary_kwargs, binary_env_changes = \
                    incorporate_fastrtps_profile(
                        binary_kwargs, binary_env_changes, profile_name
                    )
    
    native.py_binary(
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
    native.py_binary(name = name, **kwargs)

def ros_py_test(name, rmw_implementation = None, **kwargs):
    """
    Builds a Python test, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the py_test() rule.
    """
    test_name = "_" + name
    test_kwargs = dict(kwargs)
    test_kwargs.update(
        tags = ["manual"] + test_kwargs.get("tags", []))
    test_env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        test_kwargs, test_env_changes = \
            incorporate_rmw_implementation(
                test_kwargs, test_env_changes,
                rmw_implementation = rmw_implementation,
            )
        if "fastrtps" in rmw_implementation:
            if "block-network" in test_kwargs.get("tags", []):
                profile_name = name + ".fastrtps_profile.xml"
                generate_isolated_fastrtps_profile(profile_name)
                test_kwargs, test_env_changes = \
                    incorporate_fastrtps_profile(
                        test_kwargs, test_env_changes, profile_name
                    )

    native.py_test(
        name = test_name,
        **test_kwargs
    )

    shim_name = test_name + "_shim.py"
    shim_kwargs = keep_common(kwargs)
    shim_kwargs.update(testonly = True)
    dload_py_shim(
        name = shim_name,
        target = ":" + test_name,
        env_changes = test_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        data = [":" + test_name],
        deps = ["@bazel_tools//tools/python/runfiles"],
        tags = ["nolint"] + kwargs.get("tags", [])
    )
    native.py_test(name = name, **kwargs)
