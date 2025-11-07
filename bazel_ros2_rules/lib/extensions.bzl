# -*- python -*-

load("//lib:repos.bzl", "ros2_local_repository")
load("//lib/private:utilities.bzl", "find_local_ros2_distribution")

def _local_ros2_implementation(module_ctx):
    allowed_system_libs = []
    include_packages = []
    exclude_packages = []
    overlays = []
    jobs = 0

    for module in module_ctx.modules:
        if len(module.tags.distribution) > 1:
            fail(
                "There can only be one local " +
                "ROS 2 distribution per workspace",
            )
        distro = module.tags.distribution[0]
        allowed_system_libs += distro.allowed_system_libs
        include_packages += distro.include_packages
        exclude_packages += distro.exclude_packages
        if len(distro.overlays) > 0:
            if not module.is_root:
                fail("Only the root module can specify ROS 2 overlays")
            overlays += distro.overlays
        if distro.jobs != 0:
            if jobs != 0:
                jobs = min(distro.jobs, jobs)
            else:
                jobs = distro.jobs

    if include_packages:
        include_packages += [
            "action_msgs",
            "builtin_interfaces",
            "ros2cli_common_extensions",
            "ros2cli",
            "rosidl_default_generators",
            "service_msgs",
            "unique_identifier_msgs",
        ]

    underlay = find_local_ros2_distribution(module_ctx)

    ros2_local_repository(
        name = "local_ros2",
        include_packages = include_packages,
        exclude_packages = exclude_packages,
        workspaces = [underlay] + overlays,
        jobs = jobs,
        allowed_system_libs = allowed_system_libs,
    )

# NOTE: This precludes multi-distro setups. That is, there can only be one
# ROS 2 distro in a given Bazel workspace, and all Bazel modules that rely
# on ROS 2 must use that same distribution.
local_ros2 = module_extension(
    implementation = _local_ros2_implementation,
    doc = """
Scrapes a local ROS 2 installation and binds it to a "local_ros2" repository.
This extension will search for a ROS 2 installation under /opt/ros. If there
is more than one, ROS_DISTRO must be set to disambiguate. If the ROS 2
installation is to be found elsewhere, the ROS_DISTRO_PREFIX may be used.
Modules may control how the installation is scraped using the "distribution"
tag. Only one tag per module may be specified. All modules in the workspace
will share the same installation.
""",
    tag_classes = {
        "distribution": tag_class(attrs = {
            "include_packages": attr.string_list(
                doc = "Optional set of packages to include, " +
                      "with its recursive dependencies. Defaults to all.",
            ),
            "exclude_packages": attr.string_list(
                doc = "Optional set of packages to exclude, " +
                      "with precedence over included packages. " +
                      "Defaults to none.",
            ),
            "overlays": attr.string_list(
                doc = "Paths to ROS 2 workspace install trees. " +
                      "Each workspace specified overlays the previous one." +
                      "Only the root module may specify overlays.",
            ),
            "jobs": attr.int(
                doc = "Number of CMake jobs to use during package " +
                      "configuration and scrapping. " +
                      "Defaults to using all cores.",
                default = 0,
            ),
            "allowed_system_libs": attr.string_list(
                doc = "Optional list of regular expressions (strings) to " +
                      "whitelist system libraries for the scraping tool. " +
                      "Each entry should be a regular expression matching " +
                      "the library basename (e.g. libfoo\\.so[.0-9]*).",
                default = [],
            ),
        }),
    },
)
