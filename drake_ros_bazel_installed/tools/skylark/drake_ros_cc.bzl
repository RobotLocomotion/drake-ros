# -*- python -*-

load(
    "//tools/skylark:dload_cc.bzl",
    "dload_aware_cc_library",
    "dload_cc_shim",
)

def drake_ros_cc_library(name, deps = [], data = [], runenv = {},
                         testonly = False, visibility = None, **kwargs):
    """
    Builds a C/C++ library and carries runtime information, if any.

    Equivalent to the cc_library() rule.
    """
    library = "_" + name
    native.cc_library(
        name = library,
        deps = deps,
        data = data,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
    dload_aware_cc_library(
        name = name,
        base = ":" + library,
        data = data,
        deps = [":" + library] + deps,
        runenv = runenv,
        testonly = testonly,
        visibility = visibility,
    )

def drake_ros_cc_binary_import(name, executable, data = [], deps = [], tags = [],
                               testonly = 0, visibility = None, **kwargs):
    """
    Imports a pre-compiled C/C++ executable, picking up runtime information
    in dependencies.

    Args:
        executable: executable file

    Similar to the cc_binary() rule.
    """
    shim = name + "_shim.cc"
    dload_cc_shim(
        name = shim,
        target = executable,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.cc_binary(
        name = name,
        srcs = [shim],
        data = [executable] + data,
        deps = [
            "@bazel_tools//tools/cpp/runfiles",
        ] + deps,
        tags = ["nolint"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )

def drake_ros_cc_binary(name, srcs = [], data = [], deps = [], linkshared = None, tags = [],
                        testonly = False, visibility = None, **kwargs):
    """
    Builds a C/C++ binary, picking up runtime information in dependencies.

    Equivalent to the cc_binary() rule.

    Propagation of runtime information is disabled if linkshared is True.
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

def drake_ros_cc_test(name, srcs = [], data = [], deps = [],
                      testonly = 1, visibility = [], tags = [],
                      **kwargs):
    """
    Builds C/C++ test, picking up runtime information in dependencies.

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
