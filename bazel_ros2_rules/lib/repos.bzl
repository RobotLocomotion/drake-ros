# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch", "update_attrs")
load("//lib/private:repos.bzl", "base_ros2_repository", "base_ros2_repository_attributes")

_ros2_local_repository_attrs = {
    "workspaces": attr.string_list(
        doc = "Paths to ROS 2 workspace install trees. " +
              "Each workspace specified overlays the previous one." + 
              "If none are provided, ROS_DISTRO_PREFIX will be used to " + 
              "locate a workspace. If none is set, ROS_DISTRO will be used " +
              "to locate a workspace under /opt/ros. Else, the latest LTS distro " +
              "will be looked up.",
    ),
}
_ros2_local_repository_attrs.update(base_ros2_repository_attributes())

def _ros2_local_repository_impl(repo_ctx):
    workspaces = []
    if not repo_ctx.attr.workspaces:
        repo_ctx.report_progress("Searching for ROS 2 underlays")
        prefix = repo_ctx.getenv("ROS_DISTRO_PREFIX")
        if not prefix:
            distro = repo_ctx.getenv("ROS_DISTRO")
            if not distro:
                distro = "jazzy"
            prefix = "/opt/ros/" + distro
        workspaces.append(prefix)
    else:
        workspaces.extend(repo_ctx.attr.workspaces)

    repo_ctx.report_progress("Sandboxing ROS 2 workspaces")
    workspaces_in_sandbox = {}
    for path in workspaces:
        path = path.rstrip("/")
        path_in_sandbox = path.replace("/", "_")
        repo_ctx.symlink(path, path_in_sandbox)
        workspaces_in_sandbox[path] = path_in_sandbox

    base_ros2_repository(repo_ctx, workspaces_in_sandbox)

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
    "sha256_url": attr.string(
        doc = "The URL to fetch the SHA-256 checksum file for the tarball. " +
              "If 'sha256' is also supplied, then it must exactly match the " +
              "value fetched from this URL. " +
              "(In general, you may only want to supply one of these.)",
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
        default = "",
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
_ros2_archive_attrs.update(base_ros2_repository_attributes())

def _ros2_archive_impl(repo_ctx):
    sha256 = repo_ctx.attr.sha256
    if repo_ctx.attr.sha256_url:
        checksum_download_info = repo_ctx.download(
            repo_ctx.attr.sha256_url,
            "archive.sha256",
        )
        sha256_file = repo_ctx.read("archive.sha256").split(" ", 1)[0]
        if sha256 and sha256 != sha256_file:
            fail("sha256 and sha256_url are both supplied and are different")
        sha256 = sha256_file

    repo_ctx.report_progress("Pulling archive")
    download_info = repo_ctx.download_and_extract(
        repo_ctx.attr.url,
        "archive",
        sha256,
        repo_ctx.attr.type,
        repo_ctx.attr.strip_prefix,
    )
    patch(repo_ctx)

    workspaces_in_sandbox = {
        str(repo_ctx.path("archive")): "archive",
    }
    base_ros2_repository(repo_ctx, workspaces_in_sandbox)

    return update_attrs(
        repo_ctx.attr,
        _ros2_archive_attrs.keys(),
        {"sha256": download_info.sha256},
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
