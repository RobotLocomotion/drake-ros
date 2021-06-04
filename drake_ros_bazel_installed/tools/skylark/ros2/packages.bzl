
def package_share_filegroup(name, share_directories):
    native.filegroup(
        name = name,
        srcs = [path for path in native.glob(
            include = ["{}/**".format(dirpath) for dirpath in share_directories],
            exclude = [
                "*/cmake/**",
                "*/environment/**",
                "*/*.sh",
                "*/*.bash",
                "*/*.dsv",
            ]
        ) if " " not in path],
        # NOTE(hidmic): workaround lack of support for spaces.
        # See https://github.com/bazelbuild/bazel/issues/4327.
    )


def package_interfaces_filegroup(name, share_directory):
    native.filegroup(
        name = name + "_defs",
        srcs = native.glob(include = [
            "{}/**/*.idl".format(share_directory),
            "{}/**/*.msg".format(share_directory),
            "{}/**/*.srv".format(share_directory),
            "{}/**/*.action".format(share_directory),
        ])
    )
