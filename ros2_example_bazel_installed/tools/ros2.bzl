# -*- python -*-

load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_archive")
load("@bazel_ros2_rules//ros2:defs.bzl", "ros2_local_repository")

def ros2_repository(name, prefix, **kwargs):
    if prefix == "":
        ros2_archive(
            name = name,
            # TODO(hidmic): mirror this tarball for stability
            url = "http://repo.ros2.org/ci_archives/rolling-on-focal/ros2-rolling-linux-focal-amd64-ci.tar.bz2",
            # NOTE: the following checksum is likely to be often invalidated
            # (as of Feb 2022, each time the preceding tarball gets rebuilt)
            sha256 = "5bd37168cd2b704eabee207145362ce38eb83431ac183f9a9b432121c9fb8015",
            strip_prefix = "ros2-linux",
            # Fix wrong chained prefix, if any
            patch_cmds = ["sed -i 's|COLCON_CURRENT_PREFIX=\"/opt/ros/rolling\"||g' archive/setup.sh"],
            **kwargs
        )
        return

    ros2_local_repository(
        name = name,
        workspaces = [prefix],
        **kwargs
    )
