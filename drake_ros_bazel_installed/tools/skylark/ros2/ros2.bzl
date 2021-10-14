# -*- python -*-

load("@drake_ros//tools/skylark:execute.bzl", "execute_or_fail")

MANIFEST = [
    "cmake_tools/packages.py",
    "cmake_tools/server_mode.py",
    "cmake_tools/__init__.py",

    "resources/bazel/common.bzl.tpl",
    "resources/bazel/distro.bzl.tpl",
    "resources/bazel/ros_cc.bzl.tpl",
    "resources/bazel/ros_py.bzl.tpl",
    "resources/bazel/rosidl.bzl.tpl",
    "resources/bazel/snippets/overlay_executable.bazel.tpl",
    "resources/bazel/snippets/package_interfaces_filegroup.bazel.tpl",
    "resources/bazel/snippets/package_cc_library.bazel.tpl",
    "resources/bazel/snippets/package_meta_py_library.bazel.tpl",
    "resources/bazel/snippets/package_py_library.bazel.tpl",
    "resources/bazel/snippets/package_py_library_with_cc_libs.bazel.tpl",
    "resources/bazel/snippets/package_share_filegroup.bazel.tpl",
    "resources/bazel/snippets/prologue.bazel.tpl",

    "resources/cmake/ament_cmake_CMakeLists.txt.in",

    "ros2bzl/utilities.py",
    "ros2bzl/sandboxing.py",
    "ros2bzl/resources.py",
    "ros2bzl/templates.py",
    "ros2bzl/__init__.py",
    "ros2bzl/scrapping/system.py",
    "ros2bzl/scrapping/metadata.py",
    "ros2bzl/scrapping/ament_python.py",
    "ros2bzl/scrapping/ament_cmake.py",
    "ros2bzl/scrapping/__init__.py",
]

def _label(relpath):
    return Label("//tools/skylark/ros2:" + relpath)

def _impl(repo_ctx):
    for relpath in MANIFEST:
        repo_ctx.symlink(_label(relpath), relpath)

    repo_ctx.template(
        "setup.sh", _label("resources/shell/setup.sh.in"),
        substitutions = {
            "@REPOSITORY_DIR@": str(repo_ctx.path(".")),
            "@WORKSPACES@": " ".join(repo_ctx.attr.workspaces),
            "@CMAKE_PREFIX_PATH@": ":".join(repo_ctx.attr.workspaces),
        },
        executable = True
    )

    generate_tool = repo_ctx.path(_label("generate_repository_files.py"))
    cmd = ["./setup.sh", str(generate_tool)]
    for ws in repo_ctx.attr.workspaces:
        ws_in_sandbox = ws.replace("/", "_")
        cmd.extend(["-s", ws + ":" + ws_in_sandbox])
    for pkg in repo_ctx.attr.include_packages:
        cmd.extend(["-i", pkg])
    for pkg in repo_ctx.attr.exclude_packages:
        cmd.extend(["-e", pkg])
    for target, data in repo_ctx.attr.extra_data.items():
        for label in data:
            cmd.extend(["-x", target + ".data+=" + label])
    if repo_ctx.attr.jobs > 0:
        cmd.extend(["-j", repr(repo_ctx.attr.jobs)])
    cmd.append(repo_ctx.name)
    execute_or_fail(repo_ctx, cmd, quiet=True)

ros2_local_repository = repository_rule(
    attrs = dict(
        workspaces = attr.string_list(mandatory = True),
        include_packages = attr.string_list(),
        exclude_packages = attr.string_list(),
        extra_data = attr.string_list_dict(),
        jobs = attr.int(default=0),
    ),
    local = False,
    implementation = _impl,
)
"""
Scraps ROS 2 workspaces and exposes its artifacts as a Bazel local repository.

Args:
    workspaces: paths to ROS 2 workspace install trees. Each workspace specified overlays the previous one.
    include_packages: optional set of packages to include, with its recursive dependencies. Defaults to all.
    exclude_packages: optional set of packages to exclude, with precedence over included files. Defaults to none.
    extra_data: optional extra data dependencies for given targets
    jobs: number of CMake jobs to use during package configuration and scrapping. Defaults to using all cores.
"""
