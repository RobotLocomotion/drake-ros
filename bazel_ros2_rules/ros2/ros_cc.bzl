# -*- python -*-

load(
    "//tools:dload_cc.bzl",
    "dload_cc_main",
)
load(
    "//tools:kwargs.bzl",
    "filter_to_only_common_kwargs",
    "remove_test_specific_kwargs",
)
load(
    "//tools:common.bzl",
    "incorporate_rmw_implementation",
)
load("//tools:ament_index.bzl", "AmentIndex")
load(
    ":distro.bzl",
    "RUNTIME_ENVIRONMENT",
)

def ros_cc_binary(
        name,
        rmw_implementation = None,
        cc_binary_rule = native.cc_binary,
        cc_library_rule = native.cc_library,
        **kwargs):
    """
    Builds a C/C++ binary and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the cc_binary() rule, which this rule decorates.

    Args:
        name: C/C++ binary target name
        rmw_implementation: optional RMW implementation to run against
        cc_binary_rule: optional cc_binary() rule override
        cc_library_rule: optional cc_library() rule override

    Additional keyword arguments are forwarded to the `cc_binary_rule`.

    This wrapper does not support programs that use the 3-argument form of
    `main` (where environment variables are passed as a third argument).
    """

    # When creating a shared library ("libfoo.so"), don't do anything special.
    if kwargs.get("linkshared", False):
        native.cc_binary(name = name, **kwargs)
        return

    # Prepare the list of environment actions.
    env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        kwargs, env_changes = \
            incorporate_rmw_implementation(
                kwargs,
                env_changes,
                rmw_implementation = rmw_implementation,
            )

    # Declare an unshimmed, manual binary. This might be helpful for debugging,
    # but anyway is required to consolidate the program's ROS-Bazel aspects
    # into one place.
    noshim_name = "_{}_noshim_manual".format(name)
    noshim_kwargs = dict(kwargs)
    noshim_kwargs.update(visibility = ["//visibility:private"])
    noshim_tags = noshim_kwargs.pop("tags", [])
    if "manual" not in noshim_tags:
        noshim_tags.append("manual")
    cc_binary_rule(
        name = noshim_name,
        tags = noshim_tags,
        **noshim_kwargs
    )

    # Codegen a main function that performs the actions.
    main_name = "_{}_wrap_main".format(name)
    main_cc = main_name + ".cc"
    dload_cc_main(
        name = main_cc,
        target = ":" + noshim_name,
        env_changes = env_changes,
        testonly = kwargs.get("testonly", False),
        tags = [
            # Only codegen when needed as a dependency.
            "manual",
        ],
        visibility = ["//visibility:private"],
    )

    # Link the main function into a static library.
    main_kwargs = filter_to_only_common_kwargs(kwargs)
    main_kwargs.update(visibility = ["//visibility:private"])
    main_tags = main_kwargs.pop("tags", [])
    if "nolint" not in main_tags:
        # Don't check generated code.
        main_tags.append("nolint")
    cc_library_rule(
        name = main_name,
        srcs = [main_cc],
        linkstatic = True,
        deps = ["@bazel_ros2_rules//ros2:apply_environment_actions"],
        tags = main_tags,
        **main_kwargs
    )

    # Link the final binary, using the codegen'd main in lieu of the original.
    kwargs.update(
        deps = kwargs.get("deps", []) + [":" + main_name],
        linkopts = kwargs.get("linkopts", []) + ["-Wl,--wrap=main"],
    )
    cc_binary_rule(name = name, **kwargs)

def ros_cc_test(
        name,
        rmw_implementation = None,
        cc_binary_rule = native.cc_binary,
        cc_library_rule = native.cc_library,
        cc_test_rule = native.cc_test,
        **kwargs):
    """
    Builds a C/C++ test and wraps it with a shim that will inject the minimal
    runtime environment necessary for execution when depending on targets from
    this ROS 2 local repository.

    Equivalent to the cc_test() rule.

    Args:
        name: C/C++ test target name
        rmw_implementation: optional RMW implementation to run against
        cc_library_rule: optional cc_library() rule override
        cc_test_rule: optional cc_test() rule override

    Additional keyword arguments are forwarded to the `cc_test_rule`.

    This wrapper does not support programs that use the 3-argument form of
    `main` (where environment variables are passed as a third argument).
    """

    # N.B. We ignore cc_binary_rule. It was used previously, but isn't anymore.
    kwargs.pop("testonly", None)
    ros_cc_binary(
        name,
        rmw_implementation = rmw_implementation,
        cc_library_rule = cc_library_rule,
        cc_binary_rule = cc_test_rule,
        testonly = True,
        **kwargs
    )
