# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch", "update_attrs")

load("//tools:execute.bzl", "execute_or_fail")

COMMON_FILES_MANIFEST = [
    "resources/ros_cc.bzl",
    "resources/ros_py.bzl",
    "resources/rosidl.bzl",

    "resources/rmw_isolation/__init__.py",
    "resources/rmw_isolation/generate_isolated_rmw_env.py",
    "resources/rmw_isolation/package.BUILD.bazel",
    "resources/rmw_isolation/rmw_isolation.cc",
    "resources/rmw_isolation/rmw_isolation.h",
    "resources/rmw_isolation/rmw_isolation.py",
    "resources/rmw_isolation/test/isolated_listener.cc",
    "resources/rmw_isolation/test/isolated_listener.py",
    "resources/rmw_isolation/test/isolated_talker.cc",
    "resources/rmw_isolation/test/isolated_talker.py",
    "resources/rmw_isolation/test/rmw_isolation_test.sh",

    "resources/tools/common.bzl",
    "resources/tools/dload.bzl",
    "resources/tools/dload_cc.bzl",
    "resources/tools/dload_py.bzl",
    "resources/tools/kwargs.bzl",
    "resources/tools/package.BUILD.bazel",
]

def _symlink_common_files(repo_ctx):
    repo_ctx.report_progress("Symlinking common files")
    for file_ in repo_ctx.attr._common_files:
        target = file_.name[len("resources/"):]
        if target.endswith("package.BUILD.bazel"):
            directory = target[:-len("package.BUILD.bazel")]
            target = directory + "BUILD.bazel"
        repo_ctx.symlink(file_, target)

def _populate_distro_specific_files(repo_ctx, workspaces):
    repo_ctx.report_progress("Generating run_under.bash")

    repo_ctx.template(
        "run.bash",
        repo_ctx.attr._run_template_file,
        substitutions = {"@WORKSPACES@": " ".join(workspaces)},
        executable = True
    )

    repo_ctx.report_progress("Generating distro_metadata.json")
    path_to_scrape_distribution_tool = repo_ctx.path(
        repo_ctx.attr._scrape_distribution_tool)
    cmd = ["./run.bash", str(path_to_scrape_distribution_tool)]
    for package in repo_ctx.attr.include_packages:
        cmd.extend(["-i", package])
    for package in repo_ctx.attr.exclude_packages:
        cmd.extend(["-e", package])
    result = execute_or_fail(repo_ctx, cmd, quiet=True)
    if result.stderr:
        print(result.stderr)
    repo_ctx.file("distro_metadata.json", result.stdout)

    repo_ctx.report_progress("Generating distro.bzl")
    path_to_generate_distro_file_tool = repo_ctx.path(
        repo_ctx.attr._generate_distro_file_tool)
    cmd = ["./run.bash", str(path_to_generate_distro_file_tool)]
    for path, path_in_sandbox in workspaces.items():
        cmd.extend(["-s", path + ":" + path_in_sandbox])
    cmd.extend(["-d", "distro_metadata.json", repo_ctx.name])
    result = execute_or_fail(repo_ctx, cmd, quiet=True)
    if result.stderr:
        print(result.stderr)
    repo_ctx.file("distro.bzl", result.stdout)

    repo_ctx.report_progress("Generating BUILD.bazel")
    path_to_generate_build_file_tool = repo_ctx.path(
        repo_ctx.attr._generate_build_file_tool)
    cmd = ["./run.bash", str(path_to_generate_build_file_tool)]
    for path, path_in_sandbox in workspaces.items():
        cmd.extend(["-s", path + ":" + path_in_sandbox])
    if repo_ctx.attr.jobs > 0:
        cmd.extend(["-j", repr(repo_ctx.attr.jobs)])
    cmd.extend(["-d", "distro_metadata.json", repo_ctx.name])
    result = execute_or_fail(repo_ctx, cmd, quiet=True)
    if result.stderr:
        print(result.stderr)
    repo_ctx.file("BUILD.bazel", result.stdout)

    repo_ctx.report_progress("Generating system-rosdep-keys.txt")
    path_to_compute_system_rosdeps_tool = repo_ctx.path(
        repo_ctx.attr._compute_system_rosdeps_tool)
    cmd = [str(path_to_compute_system_rosdeps_tool), "distro_metadata.json"]
    result = execute_or_fail(repo_ctx, cmd, quiet=True)
    if result.stderr:
        print(result.stderr)
    repo_ctx.file("system-rosdep-keys.txt", result.stdout)

def _label(relpath):
    return "//ros2:" + relpath

_base_ros2_repository_rule_attrs = {
    "include_packages": attr.string_list(
        doc = "Optional set of packages to include, " +
        "with its recursive dependencies. Defaults to all."
    ),
    "exclude_packages": attr.string_list(
        doc = "Optional set of packages to exclude, " +
        "with precedence over included packages. Defaults to none."
    ),
    "jobs": attr.int(
        doc = "Number of CMake jobs to use during package " +
        "configuration and scrapping. Defaults to using all cores.",
        default=0,
    ),
    # NOTE: all these labels are listed as private attributes
    # to force prefetching, or else repository rules will be
    # restarted on first hit. See cdc99afc1a03ff8fbbbae088d358b7c029e0d232
    # at https://github.com/bazelbuild/bazel for further reference.
    "_common_files": attr.label_list(
        default = [_label(path) for path in COMMON_FILES_MANIFEST],
        doc = "List of common files to be symlinked to every new repository."
    ),
    "_run_template_file": attr.label(
        default = _label("resources/templates/run.bash.in"),
        doc = "Template script file to run executables " +
        "in distribution environments."
    ),
    "_scrape_distribution_tool": attr.label(
        default = _label("scrape_distribution.py"),
        doc = "Tool to scrape target distribution metadata."
    ),
    "_generate_distro_file_tool": attr.label(
        default = _label("generate_distro_file.py"),
        doc = "Tool to generate distro.bzl file from distribution metadata."
    ),
    "_generate_build_file_tool": attr.label(
        default = _label("generate_build_file.py"),
        doc = "Tool to generate BUILD.bazel file from distribution metadata."
    ),
    "_compute_system_rosdeps_tool": attr.label(
        default = _label("compute_system_rosdeps.py"),
        doc = "Tool to compute system rosdep keys for target distribution."
    ),
}

_ros2_local_repository_attrs = {
    "workspaces": attr.string_list(
        doc = "Paths to ROS 2 workspace install trees. " +
        "Each workspace specified overlays the previous one.",
        mandatory = True,
    ),
}
_ros2_local_repository_attrs.update(_base_ros2_repository_rule_attrs)

def _ros2_local_repository_impl(repo_ctx):
    _symlink_common_files(repo_ctx)

    repo_ctx.report_progress("Sandboxing ROS 2 workspaces")
    workspaces_in_sandbox = {}
    for path in repo_ctx.attr.workspaces:
        path_in_sandbox = path.replace("/", "_")
        repo_ctx.symlink(path, path_in_sandbox)
        workspaces_in_sandbox[path] = path_in_sandbox

    _populate_distro_specific_files(
        repo_ctx, workspaces_in_sandbox)

ros2_local_repository = repository_rule(
    attrs = _ros2_local_repository_attrs,
    implementation = _ros2_local_repository_impl,
    doc = """
Scrapes ROS 2 workspaces in the local filesystem
and binds their resulting overlay to a Bazel repository.
""",
    local = False,
)

_ros2_archive_attrs = {
    "url": attr.string(
        doc = "The URL to fetch the ROS 2 distribution tarball from.",
        mandatory = True,
    ),
    "sha256": attr.string(
        doc = "The expected SHA-256 of the file downloaded.",
    ),
    "strip_prefix": attr.string(
        doc = "A directory prefix to strip from the extracted files.",
    ),
    "type": attr.string(
        doc = "The archive type of the downloaded file. " +
        "Determined from the file extension by default.",
    ),
    "patches": attr.label_list(
        doc = "A list of files that are to be applied as patches",
        default = [],
    ),
    "patch_tool": attr.string(
        doc = "The patch(1) utility to use. " +
        "Uses Bazel native-patch implementation by default.",
        default = ""
    ),
    "patch_args": attr.string_list(
        doc = "The arguments given to the patch tool. Defaults to -p0.",
        default = ["-p0"],
    ),
    "patch_cmds": attr.string_list(
        doc = "Sequence of Bash commands to be applied on " +
        "Linux after patches are applied.",
        default = [],
    ),
}
_ros2_archive_attrs.update(_base_ros2_repository_rule_attrs)

def _ros2_archive_impl(repo_ctx):
    _symlink_common_files(repo_ctx)

    repo_ctx.report_progress("Pulling archive")
    download_info = repo_ctx.download_and_extract(
        repo_ctx.attr.url,
        "archive",
        repo_ctx.attr.sha256,
        repo_ctx.attr.type,
        repo_ctx.attr.strip_prefix
    )
    patch(repo_ctx)

    _populate_distro_specific_files(
        repo_ctx, {str(repo_ctx.path("archive")): "archive"})

    return update_attrs(
        repo_ctx.attr, _ros2_archive_attrs.keys(),
        {"sha256": download_info.sha256}
    )

ros2_archive = repository_rule(
    attrs = _ros2_archive_attrs,
    implementation = _ros2_archive_impl,
doc = """
Dowloads a ROS 2 distribution as a tarball,
extracts it, scrapes it, and binds it to a
Bazel repository.
""",
    local = False,
)
