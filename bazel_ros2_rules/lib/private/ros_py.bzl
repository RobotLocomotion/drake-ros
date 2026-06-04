# -*- python -*-

load(
    "@bazel_ros2_rules//lib:ament_index.bzl",
    "ament_index_share_files",
)
load(
    "@bazel_ros2_rules//lib:kwargs.bzl",
    "filter_to_only_common_kwargs",
    "remove_test_specific_kwargs",
)
load(
    "@bazel_ros2_rules//lib/dynamic_load:dload_py.bzl",
    "dload_py_shim",
)
load(
    ":common.bzl",
    "generate_file",
    "incorporate_rmw_implementation",
)
load(
    ":distro.bzl",
    "REPOSITORY_ROOT",
    "RUNTIME_ENVIRONMENT",
)

_WORKSPACE_NAME = Label(REPOSITORY_ROOT + ":ros2").workspace_name

def ros_import_binary(
        name,
        executable,
        rmw_implementation = None,
        py_binary_rule = native.py_binary,
        **kwargs):
    """
    Imports an existing executable by wrapping it with a Python shim that will
    inject the minimal runtime environment necessary for execution when
    depending on this ROS 2 local repository. Imported executables need not be
    Python -- binary executables will work the same.

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
                kwargs,
                env_changes,
                rmw_implementation = rmw_implementation,
            )

    shim_name = "_" + name + "_shim.py"
    shim_kwargs = filter_to_only_common_kwargs(kwargs)
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
            "@bazel_ros2_rules//lib/dynamic_load:dload_shim_py",
            "@bazel_ros2_rules//lib/network_isolation:network_isolation_py",
        ],
    )
    py_binary_rule(name = name, **kwargs)

def ros_py_binary(
        name,
        rmw_implementation = None,
        py_binary_rule = native.py_binary,
        **kwargs):
    """
    Builds a Python binary and wraps it with a shim that will inject the
    minimal runtime environment necessary for execution when depending on
    targets from this ROS 2 local repository.

    Equivalent to the py_binary() rule, which this rule decorates.

    Args:
        name: Python binary target name
        rmw_implementation: optional RMW implementation to run against
        py_binary_rule: optional py_binary() rule override

    Additional keyword arguments are forwarded to the `py_binary_rule`.
    """

    noshim_name = "_" + name + "_noshim"
    noshim_kwargs = dict(kwargs)
    if "main" not in noshim_kwargs:
        noshim_kwargs["main"] = name + ".py"
    shim_env_changes = dict(RUNTIME_ENVIRONMENT)

    if rmw_implementation:
        noshim_kwargs, shim_env_changes = \
            incorporate_rmw_implementation(
                noshim_kwargs,
                shim_env_changes,
                rmw_implementation = rmw_implementation,
            )

    py_binary_rule(
        name = noshim_name,
        **noshim_kwargs
    )

    shim_name = "_" + name + "_shim.py"
    shim_kwargs = filter_to_only_common_kwargs(kwargs)
    dload_py_shim(
        name = shim_name,
        target = ":" + noshim_name,
        env_changes = shim_env_changes,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        data = [":" + noshim_name],
        deps = [
            "@bazel_ros2_rules//lib/dynamic_load:dload_shim_py",
            "@bazel_ros2_rules//lib/network_isolation:network_isolation_py",
            ":" + noshim_name,  # Support py_binary being used a dependency
        ],
        tags = ["nolint"] + kwargs.get("tags", []),
    )
    py_binary_rule(name = name, **kwargs)

def _add_deps(existing, new):
    deps = list(existing)
    for dep in new:
        if dep not in deps:
            deps.append(dep)
    return deps

_LAUNCH_PY_TEMPLATE = """\
import os
import sys

from python.runfiles import runfiles as runfiles_api

assert __name__ == "__main__"
runfiles = runfiles_api.Create()
launch_file = runfiles.Rlocation({launch_respath})  # noqa
ros2_bin = runfiles.Rlocation("{ros2_rlocation}")
args = [ros2_bin, "launch", launch_file] + sys.argv[1:]
os.execv(ros2_bin, args)
"""

def _make_respath(relpath, workspace_name):
    repo = native.repository_name()
    if repo == "@":
        repo = workspace_name if workspace_name != None else native.module_name()
    pkg = native.package_name()
    if pkg != "":
        pieces = [repo, pkg, relpath]
    else:
        pieces = [repo, relpath]
    return "/".join(pieces)

def ros_launch(
        name,
        launch_file,
        args = [],
        data = [],
        deps = [],
        visibility = None,
        # Optional: overrides the Bzlmod module name as the ament package name.
        # Only needed when the desired package name differs from the module name.
        workspace_name = None,
        # Executable targets to expose via the ament resource index under
        # lib/<package_name>/, enabling
        # launch_ros.actions.Node(package=<package_name>, executable=...) to
        # find Bazel-built binaries without a colcon install space.
        # Example:
        #   executables = [":talker", ":listener"],
        executables = [],
        # Data file targets to expose via the ament resource index under
        # share/<package_name>/, enabling FindPackageShare(<package_name>)
        # and get_package_share_directory(<package_name>) to locate them.
        # Example:
        #   share = ["//my_pkg:config_files"],
        share = [],
        **kwargs):
    main = "{}_roslaunch_main.py".format(name)
    launch_respath = _make_respath(launch_file, workspace_name)

    content = _LAUNCH_PY_TEMPLATE.format(
        launch_respath = repr(launch_respath),
        ros2_rlocation = _WORKSPACE_NAME + "/ros2",
    )
    generate_file(
        name = main,
        content = content,
        visibility = ["//visibility:private"],
    )

    deps = _add_deps(
        deps,
        [
            "@bazel_ros2_rules//deps/python/runfiles",
            REPOSITORY_ROOT + ":ros2",
        ],
    )

    if executables or share:
        package_name = workspace_name if workspace_name != None else native.module_name()
        index_target = "_{}_ament_index".format(name)
        ament_index_share_files(
            name = index_target,
            package_name = package_name,
            executables = executables,
            srcs = share,
            visibility = ["//visibility:private"],
        )
        data = data + executables + share + [":" + index_target]

    data = data + [launch_file]

    if "tags" not in kwargs:
        kwargs["tags"] = []
    if "nolint" not in kwargs["tags"]:
        kwargs["tags"].append("nolint")

    ros_py_binary(
        name = name,
        main = main,
        deps = deps,
        srcs = [main],
        data = data,
        visibility = visibility,
        **kwargs
    )

_LAUNCH_TEST_MAIN_TEMPLATE = """\
import os
import sys

from python.runfiles import runfiles as runfiles_api

assert __name__ == "__main__"
runfiles = runfiles_api.Create()
launch_test_file = runfiles.Rlocation({launch_test_respath})

# This shim only ever runs inside a Bazel test sandbox. The sandbox sets
# HOME=/does_not_exist (read-only) and the project .bazelrc sets ROS_HOME to
# a similar sentinel. RCL log-dir priority: ROS_LOG_DIR > $ROS_HOME/log >
# $HOME/.ros/log. Redirect all three unconditionally to the per-test tmpdir.
_tmpdir = os.environ.get("TEST_TMPDIR", "/tmp")
os.environ["ROS_LOG_DIR"] = _tmpdir
os.environ["ROS_HOME"] = _tmpdir
os.environ["HOME"] = _tmpdir

sys.argv = ["launch_test", launch_test_file] + sys.argv[1:]

from launch_testing.launch_test import main
sys.exit(main())
"""

def ros_launch_test(
        name,
        launch_test_file,
        workspace_name = None,
        executables = [],
        share = [],
        data = [],
        deps = [],
        rmw_implementation = None,
        isolate = True,
        **kwargs):
    """
    Builds a launch_testing test and wraps it with a shim that will inject the
    minimal runtime environment necessary for execution when depending on
    targets from this ROS 2 local repository.

    The test file must export generate_test_description() and any number of
    unittest.TestCase subclasses following the launch_testing convention.

    Args:
        name: test target name
        launch_test_file: label of the launch_testing test file
        workspace_name: optional ament package name override (defaults to
            the Bzlmod module name)
        executables: executable targets to register in the ament index,
            enabling Node(package=<pkg>, executable=...) lookups
        share: data file targets to register under share/<pkg>/, enabling
            FindPackageShare(<pkg>) and get_package_share_directory(<pkg>)
        data: additional runtime data deps
        deps: additional Python deps beyond launch_testing and launch
        rmw_implementation: optional RMW implementation to run against
        isolate: whether to use network namespace isolation (default True)

    Additional keyword arguments are forwarded to ros_py_test.
    """
    main = "{}_launch_test_main.py".format(name)
    launch_test_respath = _make_respath(launch_test_file, workspace_name)

    generate_file(
        name = main,
        content = _LAUNCH_TEST_MAIN_TEMPLATE.format(
            launch_test_respath = repr(launch_test_respath),
        ),
        visibility = ["//visibility:private"],
        testonly = True,
    )

    if executables or share:
        package_name = workspace_name if workspace_name != None else native.module_name()
        index_target = "_{}_ament_index".format(name)
        ament_index_share_files(
            name = index_target,
            package_name = package_name,
            executables = executables,
            srcs = share,
            visibility = ["//visibility:private"],
        )
        data = data + executables + share + [":" + index_target]

    tags = list(kwargs.pop("tags", []))
    if "nolint" not in tags:
        tags.append("nolint")

    ros_py_test(
        name = name,
        srcs = [main],
        main = main,
        data = data + [launch_test_file],
        deps = deps + [
            "@bazel_ros2_rules//deps/python/runfiles",
            "@ros2//:launch_testing_py",
            "@ros2//:launch_py",
        ],
        rmw_implementation = rmw_implementation,
        isolate = isolate,
        tags = tags,
        **kwargs
    )

def ros_py_test(
        name,
        rmw_implementation = None,
        py_binary_rule = native.py_binary,
        py_test_rule = native.py_test,
        isolate = True,
        **kwargs):
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
    noshim_name = "_" + name + "_noshim"
    noshim_kwargs = remove_test_specific_kwargs(kwargs)
    noshim_kwargs.update(testonly = True)
    shim_env_changes = dict(RUNTIME_ENVIRONMENT)
    if rmw_implementation:
        noshim_kwargs, shim_env_changes = \
            incorporate_rmw_implementation(
                noshim_kwargs,
                shim_env_changes,
                rmw_implementation = rmw_implementation,
            )

    py_binary_rule(
        name = noshim_name,
        **noshim_kwargs
    )

    shim_name = "_" + name + "_shim.py"
    shim_kwargs = filter_to_only_common_kwargs(kwargs)
    shim_kwargs.update(testonly = True)
    dload_py_shim(
        name = shim_name,
        target = ":" + noshim_name,
        env_changes = shim_env_changes,
        isolate = isolate,
        **shim_kwargs
    )

    kwargs.update(
        srcs = [shim_name],
        main = shim_name,
        data = [":" + noshim_name],
        deps = [
            "@bazel_ros2_rules//lib/dynamic_load:dload_shim_py",
            "@bazel_ros2_rules//lib/network_isolation:network_isolation_py",
        ],
        tags = ["nolint"] + kwargs.get("tags", []),
    )
    py_test_rule(name = name, **kwargs)
