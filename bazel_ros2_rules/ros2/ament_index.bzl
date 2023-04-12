# -*- python -*-

"""
Provides bazel rules for working with an ament resource index.

See this for more information:
https://github.com/ament/ament_cmake/
blob/master/ament_cmake_core/doc/resource_index.md
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("//tools:ament_index.bzl", "AmentIndex")

def _ament_index_share_files_impl(ctx):
    # declare that a "package" with the given name exists
    package_marker_path = paths.join(
        ctx.attr.prefix,
        "share/ament_index/resource_index/packages/",
        ctx.attr.package_name,
    )
    package_marker_out = ctx.actions.declare_file(package_marker_path)
    ctx.actions.write(
        output = package_marker_out,
        content = "",
    )

    # Symlink sources into the share directory
    runfiles_symlinks = {
        package_marker_path: package_marker_out,
    }

    for src in ctx.attr.srcs:
        # src is a target that could have multiple files
        for file in src.files.to_list():
            sp = file.short_path
            if sp.startswith(ctx.attr.strip_prefix):
                sp = sp[len(ctx.attr.strip_prefix):]
            symlink_path = paths.join(
                ctx.attr.prefix,
                "share",
                ctx.attr.package_name,
                sp,
            )
            runfiles_symlinks[symlink_path] = file

    return [
        AmentIndex(prefix = ctx.attr.prefix),
        DefaultInfo(
            runfiles = ctx.runfiles(root_symlinks = runfiles_symlinks),
        ),
    ]

ament_index_share_files = rule(
    attrs = dict(
        package_name = attr.string(mandatory = True),
        srcs = attr.label_list(
            mandatory = True,
            allow_empty = False,
            allow_files = True,
        ),
        # A prefix is required because the shim can't prepend the runfiles
        # root to AMENT_PREFIX_PATH
        prefix = attr.string(default = "ament_index_share_files"),
        strip_prefix = attr.string(default = ""),
    ),
    implementation = _ament_index_share_files_impl,
    output_to_genfiles = True,
    provides = [AmentIndex],
)
"""
Creates an ament resource index and share/ directory for a single package.

This creates both a package marker for a package, and a share directory
for that package.
Files in `srcs` will be added to `share/package_name`.
A target depending on this one will be able to access the files in the
share directory using the package's share directory joined with the
files they expect.

    d = ament_index_cpp::get_package_share_directory("package_name")
    file = join(d, "short_path of file")

Note that this works only when bazel creates runfiles links,
and not just a *.runfiles_manifest (which it might do on an unsupported
platform like Windows, or with `--nobuild_runfile_links`).

Note that the same `package_name` for ROS 2 packages should not be used
in multiple different rules spread across multiple Bazel packages.
This is called "overriding", and it's not currently supported.
Currently if the two calls to `ament_index_share_files` also have
the same value for "prefix", then a target depending on both may
be able to find resources from both rules, but that behavior is not
guaranteed.
TODO(sloretz) detect and error when a target is given two ament indexes
with the same package.

Args:
    package_name: name of a ROS 2 package to which these share files belong
    srcs: files to put into the share directory
    prefix: optional prefix to give to the generated runfiles.
    strip_prefix: optional prefix to strip from the short_path of the files

Provides:
    AmentIndex: a prefix path where the ament resource index was generated.
    DefaultInfo: folders and symlinked files to add to the runfiles of a
      dependent target.
"""
