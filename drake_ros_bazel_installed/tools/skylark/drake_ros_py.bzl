# -*- python -*-
# vi: set ft=python :

load(
    "//tools/skylark:dload_py.bzl",
    "dload_aware_py_library",
    "dload_py_shim",
)

def drake_ros_py_library(name, srcs = [], main = None, imports = [],
                         srcs_version = None, deps = [], data = [],
                         runenv = {}, testonly = 0, visibility = None,
                         **kwargs):
    """
    Adds a Python library and carries runtime information, if any.

    Equivalent to the py_library() rule.
    """
    library_name = "_" + name
    py_library(
        name = library_name,
        srcs = srcs,
        main = main,
        imports = imports,
        srcs_version = srcs_version,
        deps = deps,
        data = data,
        testonly = testonly,
        visibility = visibility,
        **kwargs
    )
    dload_aware_py_library(
        name = name,
        base = ":" + library_name,
        data = data,
        deps = [":" + library_name] + deps,
        runenv = runenv,
        testonly = testonly,
        visibility = visibility,
    )

def drake_ros_py_binary(name, srcs = [], main = None, imports = [], args = [],
                        srcs_version = None, data = [], deps = [], tags = [],
                        testonly = 0, visibility = None, **kwargs):
    """
    Adds a Python executable, picking up runtime information in dependencies.

    Equivalent to the py_library() rule.
    """
    binary = "_" + name
    if not main:
        main = name + ".py"
    py_binary(
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
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    py_binary(
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

def drake_ros_py_test(name, srcs = [], main = None, imports = [],
                      srcs_version = None, data = [], args = [],
                      deps = [], tags = [], visibility = None,
                      testonly = 1, **kwargs):
    """
    Adds a Python test, picking up runtime information in dependencies.

    Equivalent to the py_test() rule.
    """
    if not srcs:
        srcs = ["test/%s.py" % name]

    if not main:
        if len(srcs) > 1:
            fail("You must specify main if you have more than one source.")
        main = srcs[0]

    test = "_" + name

    py_test(
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
        data = data,
        deps = deps,
        testonly = testonly,
        visibility = visibility,
    )
    py_test(
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
