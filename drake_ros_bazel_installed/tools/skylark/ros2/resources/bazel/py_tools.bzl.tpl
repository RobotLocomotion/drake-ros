# -*- python -*-

load("@REPOSITORY_ROOT@:distro.bzl", "RUNTIME_ENVIRONMENT")
load("@drake_ros//tools/skylark:dload_py.bzl", "dload_py_shim")

def ros_py_import(
    name, executable = [], imports = [], args = [],
    data = [], deps = [], tags = [], testonly = 0,
    visibility = None, **kwargs
):
    """
    Imports an executable, injecting the runtime environment
    specific to this ROS overlay.

    Args:
        executable: executable file

    Akin to the cc_import() rule.
    """
    shim = name + "_shim.py"
    dload_py_shim(
        name = shim,
        target = executable,
        env = RUNTIME_ENVIRONMENT,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.py_binary(
        name = name,
        srcs = [shim],
        main = shim,
        imports = imports,
        data = [executable] + data,
        deps = [
            "@bazel_tools//tools/python/runfiles",
        ] + deps,
        tags = ["nolint"] + tags,
        testonly = testonly,
        visibility = visibility,
        python_version = "PY3",
        **kwargs
    )

def ros_py_binary(
    name, srcs = [], main = None, imports = [], args = [],
    srcs_version = None, data = [], deps = [], tags = [],
    testonly = 0, visibility = None, **kwargs
):
    """
    Adds a Python executable, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the py_library() rule.
    """
    binary = "_" + name
    if not main:
        if len(srcs) == 1:
            main = srcs[0]
        else:
            main = name + ".py"
    native.py_binary(
        name = binary,
        srcs = srcs,
        main = main,
        imports = imports,
        srcs_version = srcs_version,
        data = data,
        deps = deps,
        tags = tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
    shim = binary + "_shim.py"
    dload_py_shim(
        name = shim,
        target = ":" + binary,
        env = RUNTIME_ENVIRONMENT,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.py_binary(
        name = name,
        srcs = [shim],
        main = shim,
        data = [":" + binary],
        deps = [
            "@bazel_tools//tools/python/runfiles",
            ":" + binary,  # Support py_binary being used a dependency
        ],
        srcs_version = srcs_version,
        tags = ["nolint"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )

def ros_py_test(
    name, srcs = [], main = None, imports = [], srcs_version = None,
    data = [], args = [], deps = [], tags = [], visibility = None,
    testonly = 1, **kwargs
):
    """
    Adds a Python test, injecting the runtime environment
    specific to this ROS overlay.

    Equivalent to the py_test() rule.
    """
    if not srcs:
        srcs = ["test/%s.py" % name]

    if not main:
        if len(srcs) > 1:
            fail("You must specify main if you have more than one source.")
        main = srcs[0]

    test = "_" + name

    native.py_test(
        name = test,
        srcs = srcs,
        main = main,
        args = args,
        imports = imports,
        srcs_version = srcs_version,
        data = data,
        deps = deps,
        tags = ["manual"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
    shim = test + "_shim.py"
    dload_py_shim(
        name = shim,
        target = ":" + test,
        env = RUNTIME_ENVIRONMENT,
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    native.py_test(
        name = name,
        srcs = [shim],
        data = [":" + test],
        main = shim,
        deps = [
            "@bazel_tools//tools/python/runfiles",
        ],
        srcs_version = srcs_version,
        tags = ["nolint"] + tags,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
