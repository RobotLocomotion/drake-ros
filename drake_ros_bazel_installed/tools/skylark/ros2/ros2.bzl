load("@drake_ros//tools/skylark:execute.bzl", "execute_or_fail")

MANIFEST = [
    "cmake_tools/packages.py",
    "cmake_tools/server_mode.py",
    "cmake_tools/__init__.py",

    "resources/bazel/cc_tools.bzl.tpl",
    "resources/bazel/common.bzl.tpl",
    "resources/bazel/distro.bzl.tpl",
    "resources/bazel/py_tools.bzl.tpl",
    "resources/bazel/rosidl_tools.bzl.tpl",
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

def _uuid(repo_ctx):
    cmd = [repo_ctx.which("python3"), "-c", "import uuid; print(uuid.uuid1())"]
    return execute_or_fail(repo_ctx, cmd, quiet=True).stdout

def _impl(repo_ctx):
    for relpath in MANIFEST:
        repo_ctx.symlink(_label(relpath), relpath)

    repo_ctx.template(
        "setup.sh", _label("resources/shell/setup.sh.in"),
        substitutions = {
            "@ID@": _uuid(repo_ctx),
            "@REPOSITORY_DIR@": str(repo_ctx.path(".")),
            "@WORKSPACES@": " ".join(repo_ctx.attr.workspaces),
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

"""
Extracts relevant properties from CMake and Python stuff for ROS 2 install
trees (see https://www.ros.org/reps/rep-0122.html). Assumes `ament_cmake`
for C++ packages and generic Python packages.
"""

# NOTE(hidmic): `ament_cmake` is not using modern CMake targets as of Dashing,
# and as of Foxy not all packages are bound to. Stick to non-standard dependency
# recollection.

# TODO(eric): How to handle licenses?
ros2_local_repository = repository_rule(
    attrs = dict(
        # Workspaces
        # - FHS install trees (incl. ABI compatible overlays).
        workspaces = attr.string_list(mandatory = True),
        # Explicit set of packages to include, with its
        # recursive dependencies. Default to all.
        include_packages = attr.string_list(),
        # Set of packages to exclude. Default to none.
        exclude_packages = attr.string_list(),
        # Extra data dependencies for targets
        extra_data = attr.string_list_dict(),
        # Maximum CMake jobs to use during configuration
        jobs = attr.int(default=0),
    ),
    local = False,
    configure = True,
    implementation = _impl,
)
