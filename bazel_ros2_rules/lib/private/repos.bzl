load("//tools/starlark:execute.bzl", "execute_or_fail")

COMMON_FILES_MANIFEST = [
    "common.bzl",
    "ros_cc.bzl",
    "ros_py.bzl",
    "rosidl.bzl",
    "cmake_tools/__init__.py",
    "cmake_tools/file_api.py",
    "cmake_tools/packages.py",
    "ros2bzl/__init__.py",
    "ros2bzl/resources.py",
    "ros2bzl/sandboxing.py",
    "ros2bzl/scraping/__init__.py",
    "ros2bzl/scraping/ament_cmake.py",
    "ros2bzl/scraping/ament_python.py",
    "ros2bzl/scraping/metadata.py",
    "ros2bzl/scraping/properties.py",
    "ros2bzl/scraping/python.py",
    "ros2bzl/scraping/system.py",
    "ros2bzl/templates.py",
    "ros2bzl/utilities.py",
    "templates/ament_cmake_CMakeLists.txt.in",
    "templates/distro.bzl.tpl",
    "templates/overlay_executable.bazel.tpl",
    "templates/package_alias.bazel.tpl",
    "templates/package_cc_library.bazel.tpl",
    "templates/package_interfaces_filegroup.bazel.tpl",
    "templates/package_meta_py_library.bazel.tpl",
    "templates/package_py_library_with_cc_libs.bazel.tpl",
    "templates/package_py_library.bazel.tpl",
    "templates/package_share_filegroup.bazel.tpl",
    "templates/prologue.bazel",
]

def base_ros2_repository(repo_ctx, workspaces):
    """
    Provides the base main logic to setup a ROS 2 repository given a
    stack of overlayed ROS 2 workspaces.

    Requires `base_ros2_repository_attrs()` to have been included in
    `repository_rule(*, attrs)`.

    Arguments:
        workspaces:
            Paths to ROS 2 workspace install trees provided as
            a mapping, from absolute paths (potentially outside
            Bazel's sandbox) to relative paths, to be resolved
            relative to the repository root directory. Iteration
            order (which mirrors insertion order) dictates the
            overlay ordering: latter workspaces overlay previous
            workspaces.
    """
    repo_ctx.report_progress("Symlinking common files")
    for file_ in repo_ctx.attr._common_files:
        repo_ctx.symlink(file_, file_.name)

    repo_ctx.report_progress("Generating run_under.bash")

    repo_ctx.template(
        "run.bash",
        repo_ctx.attr._run_template_file,
        substitutions = {"@WORKSPACES@": " ".join(workspaces)},
        executable = True,
    )

    env = {"PYTHONPATH": "."}

    # Propagate allowed system library patterns to the scraping tools.
    # The list is provided as a colon-separated string in the environment.
    # if getattr(repo_ctx.attr, 'allowed_system_libs', []):
    env["ROS2RULES_ALLOWED_SYSTEM_LIBS"] = \
        ":".join(repo_ctx.attr.allowed_system_libs)

    repo_ctx.report_progress("Generating distro_metadata.json")
    path_to_scrape_distribution_tool = repo_ctx.path(
        repo_ctx.attr._scrape_distribution_tool,
    )
    cmd = ["./run.bash", str(path_to_scrape_distribution_tool)]
    for package in repo_ctx.attr.include_packages:
        cmd.extend(["-i", package])
    for package in repo_ctx.attr.exclude_packages:
        cmd.extend(["-e", package])
    cmd.extend(["-o", "distro_metadata.json"])
    result = execute_or_fail(repo_ctx, cmd, quiet = True, environment = env)
    if result.stderr:
        print(result.stderr)

    repo_ctx.report_progress("Generating distro.bzl")
    path_to_generate_distro_file_tool = repo_ctx.path(
        repo_ctx.attr._generate_distro_file_tool,
    )
    cmd = ["./run.bash", str(path_to_generate_distro_file_tool)]
    for path, path_in_sandbox in workspaces.items():
        cmd.extend(["-s", path + ":" + path_in_sandbox])
    cmd.extend(["-d", "distro_metadata.json", repo_ctx.name])
    cmd.extend(["-o", "distro.bzl"])
    if repo_ctx.attr.default_localhost_only:
        cmd.extend(["--default-localhost-only"])
    result = execute_or_fail(repo_ctx, cmd, quiet = True, environment = env)
    if result.stderr:
        print(result.stderr)

    repo_ctx.report_progress("Generating BUILD.bazel")
    path_to_generate_build_file_tool = repo_ctx.path(
        repo_ctx.attr._generate_build_file_tool,
    )
    cmd = ["./run.bash", str(path_to_generate_build_file_tool)]
    for path, path_in_sandbox in workspaces.items():
        cmd.extend(["-s", path + ":" + path_in_sandbox])
    if repo_ctx.attr.jobs > 0:
        cmd.extend(["-j", repr(repo_ctx.attr.jobs)])
    cmd.extend(["-d", "distro_metadata.json", repo_ctx.name])
    cmd.extend(["-o", "BUILD.bazel"])
    result = execute_or_fail(repo_ctx, cmd, quiet = True, environment = env)
    if result.stderr:
        print(result.stderr)

    repo_ctx.report_progress("Generating system-rosdep-keys.txt")
    path_to_compute_system_rosdeps_tool = repo_ctx.path(
        repo_ctx.attr._compute_system_rosdeps_tool,
    )
    cmd = [
        "./run.bash",
        str(path_to_compute_system_rosdeps_tool),
        "distro_metadata.json",
    ]
    cmd.extend(["-o", "system-rosdep-keys.txt"])
    result = execute_or_fail(repo_ctx, cmd, quiet = True)
    if result.stderr:
        print(result.stderr)

def _label(relpath):
    return Label("//lib/private:" + relpath)

def base_ros2_repository_attributes():
    """
    Attributes necessary for `base_ros2_repository()`.

    Note:
        All of the `_*` labels are listed as private attributes to force
        prefetching, or else repository rules will be restarted on first hit.
        See https://github.com/bazelbuild/bazel/commit/cdc99afc1a03ff8fbbbae088d358b7c029e0d232
        and https://github.com/bazelbuild/bazel/issues/4533 for further reference.
    """  # noqa
    return {
        "include_packages": attr.string_list(
            doc = "Optional set of packages to include, " +
                  "with its recursive dependencies. Defaults to all.",
        ),
        "exclude_packages": attr.string_list(
            doc = "Optional set of packages to exclude, " +
                  "with precedence over included packages. Defaults to none.",
        ),
        "jobs": attr.int(
            doc = "Number of CMake jobs to use during package " +
                  "configuration and scrapping. Defaults to using all cores.",
            default = 0,
        ),
        "default_localhost_only": attr.bool(
            doc = "Whether ROS communication should be restricted to " +
                  "localhost by default. Defaults to True",
            default = True,
        ),
        "_common_files": attr.label_list(
            default = [_label(path) for path in COMMON_FILES_MANIFEST],
            doc = "List of common files to be symlinked to every new " +
                  "repository.",
        ),
        "_run_template_file": attr.label(
            default = _label("templates/run.bash.in"),
            doc = "Template script file to run executables " +
                  "in distribution environments.",
        ),
        "_scrape_distribution_tool": attr.label(
            default = _label("scripts/scrape_distribution.py"),
            doc = "Tool to scrape target distribution metadata.",
        ),
        "_generate_distro_file_tool": attr.label(
            default = _label("scripts/generate_distro_file.py"),
            doc = "Tool to generate distro.bzl file from distribution " +
                  "metadata.",
        ),
        "_generate_build_file_tool": attr.label(
            default = _label("scripts/generate_build_file.py"),
            doc = "Tool to generate BUILD.bazel file from distribution " +
                  "metadata.",
        ),
        "_compute_system_rosdeps_tool": attr.label(
            default = _label("scripts/compute_system_rosdeps.py"),
            doc = "Tool to compute system rosdep keys for target " +
                  "distribution.",
        ),
        "allowed_system_libs": attr.string_list(
            doc = "Optional list of regular expressions (strings) that " +
                  "will be added to the scraping tool's allowed system libs " +
                  "list. Each entry must be a valid regular expression. " +
                  "Useful to whitelist specific system libs " +
                  "(e.g. libfoo\\.so[.0-9]*).",
            default = [],
        ),
    }
