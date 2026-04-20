""" Defines a rule and macro for transforming xacro files to a URDF.
"""

load("@bazel_ros2_rules//lib:ament_index.bzl", "ament_index_share_files")
load(":distro.bzl", "REPOSITORY_ROOT")
load(":ros_py.bzl", "ros_py_binary")

# Derive the plain workspace name for use in Rlocation paths ("name/target").
# Label.workspace_name handles both Bzlmod ("@@name//") and WORKSPACE ("@name//")
# formats correctly without manual string manipulation.
_WORKSPACE_NAME = Label(REPOSITORY_ROOT + ":xacro").workspace_name

def _xacro_generate_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, ctx.attr.content, is_executable = True)
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]

_xacro_generate_file = rule(
    attrs = {"content": attr.string(mandatory = True)},
    output_to_genfiles = True,
    implementation = _xacro_generate_file_impl,
)

# Runner script template.  The outer dload shim (from ros_py_binary) has
# already set AMENT_PREFIX_PATH to include both system and user-package
# prefixes (all absolute via $RUNFILES_DIR) before this script runs.  What is
# needed then, is to locate and exec the inner @ros2//:xacro shim.
_XACRO_RUNNER_TEMPLATE = """\
import os
import sys

from python.runfiles import runfiles as runfiles_api

assert __name__ == "__main__"
runfiles = runfiles_api.Create()
xacro_bin = runfiles.Rlocation("{xacro_rlocation}")
os.execv(xacro_bin, [xacro_bin] + sys.argv[1:])
"""

def _ros_xacro_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.name + ".urdf")
    args = ctx.actions.args()
    args.add(ctx.file.src)
    args.add("-o", output)
    args.add_all(ctx.attr.xacro_args)
    ctx.actions.run(
        inputs = [ctx.file.src] + ctx.files.data,
        outputs = [output],
        executable = ctx.executable.xacro_tool,
        arguments = [args],
    )
    return [DefaultInfo(files = depset([output]))]

_ros_xacro_rule = rule(
    attrs = {
        "src": attr.label(
            allow_single_file = [".xacro"],
            mandatory = True,
            doc = "The main .urdf.xacro file to process.",
        ),
        "data": attr.label_list(
            allow_files = [".xacro"],
            default = [],
            doc = "Additional .xacro files included via relative paths.",
        ),
        "xacro_args": attr.string_list(
            default = [],
            doc = "Extra key:=value arguments forwarded to xacro.",
        ),
        "xacro_tool": attr.label(
            executable = True,
            cfg = "exec",
            mandatory = True,
            doc = "The per-invocation xacro runner binary.",
        ),
    },
    implementation = _ros_xacro_impl,
)

def ros_xacro(name, src, data = [], ros_packages = {}, xacro_args = [], visibility = None, **kwargs):
    """Transforms a .urdf.xacro file into a .urdf file.

    User-defined packages are declared inline via the ros_packages dict.  Each
    entry maps a ROS package name to the list of files to place under
    share/<package_name>/.  The strip_prefix is derived automatically from the
    calling BUILD file's package path, so files are placed relative to that
    package directory.

    The dload shim from ros_py_binary extends AMENT_PREFIX_PATH with all
    registered package prefixes (absolute, via $RUNFILES_DIR) before xacro
    runs, making $(find <pkg>) work for both local and system packages.

    Example:
        ros_xacro(
            name = "example",
            src = "robot.urdf.xacro",
            data = ["base.xacro", "arm.xacro"],
            ros_packages = {
                "my_robot": glob(["urdf/**"]),
            },
            xacro_args = ["sim:=false"],
        )

    Args:
        name:         target name; the output file is named <name>.urdf
        src:          the .urdf.xacro source file
        data:         additional .xacro files included via relative paths
                      (i.e. plain <xacro:include filename="other.xacro"/>);
                      must be listed here so Bazel sandboxes them and tracks
                      them as dependencies for incremental rebuilds
        ros_packages: dict mapping ROS package name to list of share files;
                      files are stripped of the calling package's path prefix
                      automatically before being placed under share/<pkg>/
        xacro_args:   list of key:=value arguments forwarded to xacro
        visibility:   target visibility
    """
    pkg_targets = []
    strip_prefix = native.package_name() + "/" if native.package_name() else ""
    for pkg_name, srcs in ros_packages.items():
        index_name = "_{}_pkg_{}".format(name, pkg_name)
        ament_index_share_files(
            name = index_name,
            package_name = pkg_name,
            srcs = srcs,
            strip_prefix = strip_prefix,
            visibility = ["//visibility:private"],
        )
        pkg_targets.append(":" + index_name)

    runner_data = [REPOSITORY_ROOT + ":xacro"] + pkg_targets

    runner_main = "_{}_runner_main.py".format(name)
    _xacro_generate_file(
        name = runner_main,
        content = _XACRO_RUNNER_TEMPLATE.format(
            xacro_rlocation = _WORKSPACE_NAME + "/xacro",
        ),
        visibility = ["//visibility:private"],
    )

    runner_name = "_{}_runner".format(name)
    ros_py_binary(
        name = runner_name,
        main = runner_main,
        srcs = [runner_main],
        data = runner_data,
        deps = ["@bazel_ros2_rules//deps/python/runfiles"],
        visibility = ["//visibility:private"],
    )

    _ros_xacro_rule(
        name = name,
        src = src,
        data = data,
        xacro_args = xacro_args,
        xacro_tool = ":" + runner_name,
        visibility = visibility,
    )
