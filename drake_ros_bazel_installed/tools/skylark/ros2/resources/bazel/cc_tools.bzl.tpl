# -*- python -*-

load("@REPOSITORY_ROOT@:distro.bzl", "RUNTIME_ENVIRONMENT")
load("@drake_ros//tools/skylark:dload_cc.bzl", "dload_cc_shim")

def ros_cc_binary(
    name, srcs = [], data = [], deps = [], linkshared = None,
    tags = [], testonly = False, visibility = None, **kwargs
):
    """
    Builds a C/C++ binary, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the cc_binary() rule.
    """
    if linkshared:
        native.cc_binary(
            name = name,
            srcs = srcs,
            data = data,
            deps = deps,
            linkshared = linkshared,
            visibility = visibility,
            testonly = testonly,
            tags = tags,
            **kwargs
        )
        return
    binary = "_" + name
    native.cc_binary(
        name = binary,
        srcs = srcs,
        data = data,
        deps = deps,
        linkshared = linkshared,
        visibility = visibility,
        testonly = testonly,
        tags = tags,
        **kwargs
    )

    shim = binary + "_shim.cc"
    dload_cc_shim(
        name = shim,
        target = ":" + binary,
        env = RUNTIME_ENVIRONMENT,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.cc_binary(
        name = name,
        srcs = [shim],
        data = [":" + binary],
        deps = [
            "@bazel_tools//tools/cpp/runfiles",
        ],
        tags = ["nolint"] + tags,
        visibility = visibility,
        testonly = testonly,
        **kwargs
    )

def ros_cc_test(
    name, srcs = [], data = [], deps = [],
    testonly = 1, visibility = [], tags = [],
    **kwargs
):
    """
    Builds C/C++ test, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the cc_test() rule.
    """

    if not srcs:
        srcs = ["test/%s.cc" % name]

    test = "_" + name

    native.cc_test(
        name = test,
        srcs = srcs,
        data = data,
        deps = deps,
        tags = ["manual"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
    shim = test + "_shim.cc"
    dload_cc_shim(
        name = shim,
        target = ":" + test,
        env = RUNTIME_ENVIRONMENT,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.cc_test(
        name = name,
        srcs = [shim],
        data = [":" + test],
        deps = [
            "@bazel_tools//tools/cpp/runfiles",
        ],
        tags = ["nolint"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
