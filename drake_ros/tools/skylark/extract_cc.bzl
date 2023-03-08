"""
Routines for extracting cc object files from targets.
"""

def _extract_cc_object_files_ctx(ctx, cc_info):
    # https://bazel.build/rules/lib/CcInfo
    object_files = []
    owners = []

    # https://bazel.build/rules/lib/LinkingContext
    for linker_input in cc_info.linking_context.linker_inputs.to_list():
        owner = linker_input.owner
        for lib in linker_input.libraries:
            # https://bazel.build/rules/lib/LibraryToLink#pic_objects
            object_files += lib.pic_objects
            owners += [owner] * len(lib.pic_objects)
    return depset(object_files).to_list(), owners

ExtractCcSrcs = provider(
    fields = [
        "srcs",
    ],
)

def _extract_cc_srcs_impl(target, ctx):
    srcs = getattr(ctx.rule.attr, "srcs", [])
    return [
        ExtractCcSrcs(
            srcs = srcs,
        ),
    ]

"""Extracts `srcs` Target lists for consumption by rules."""

extract_cc_srcs = aspect(
    implementation = _extract_cc_srcs_impl,
    attr_aspects = ["srcs"],
)

def extract_direct_cc_hdrs_srcs_and_transitive_data(
        name,
        deps,
        reuse_object_files,
        **kwargs):
    """
    Extracts direct C++ hdrs, srcs, and transitive data in such a way that they
    can be relinked.

    This is intended for use with recompiling / relinking targets while
    replacing certain deps (e.g. going from static libraries to shared
    libraries for Python bindnigs).
    """

    hdrs_name = "_" + name + "_hdrs"
    _do_extract_cc_srcs_hdrs_data(
        name = hdrs_name,
        mode = "direct_hdrs",
        deps = deps,
        **kwargs
    )

    srcs_name = "_" + name + "_srcs"
    if reuse_object_files:
        srcs_mode = "direct_object_files"
    else:
        srcs_mode = "direct_srcs"
    _do_extract_cc_srcs_hdrs_data(
        name = srcs_name,
        mode = srcs_mode,
        deps = deps,
        **kwargs
    )

    data_name = "_" + name + "_data"
    _do_extract_cc_srcs_hdrs_data(
        name = data_name,
        mode = "data",
        deps = deps,
        **kwargs
    )

    # Include filegroup for testing purposes.
    native.filegroup(
        name = name,
        testonly = 1,
        srcs = [
            hdrs_name,
            srcs_name,
            data_name,
        ],
    )

    return [hdrs_name], [srcs_name], [data_name]

def _do_extract_impl(ctx):
    mode = ctx.attr.mode
    if mode == "direct_srcs":
        src_depsets = []
        for dep in ctx.attr.deps:
            for src in dep[ExtractCcSrcs].srcs:
                src_depsets.append(src.files)
        return DefaultInfo(files = depset(transitive = src_depsets))
    elif mode == "direct_object_files":
        object_files = []
        for dep in ctx.attr.deps:
            new_object_files, owners = _extract_cc_object_files_ctx(
                ctx,
                cc_info = dep[CcInfo],
            )
            for object_file, owner in zip(new_object_files, owners):
                # Direct dependencies.
                if owner == dep.label:
                    object_files.append(object_file)
        return DefaultInfo(files = depset(object_files))
    elif mode == "direct_hdrs":
        hdrs = []
        for dep in ctx.attr.deps:
            hdrs += dep[CcInfo].compilation_context.direct_public_headers
        return DefaultInfo(files = depset(hdrs))
    elif mode == "data":
        data_runfiles = []
        for dep in ctx.attr.deps:
            data_runfiles.append(dep[DefaultInfo].data_runfiles)
        runfiles = ctx.runfiles().merge_all(data_runfiles)
        return DefaultInfo(runfiles = runfiles)
    else:
        fail("Bad mode")

_attrs = {
    "mode": attr.string(
        values = ["direct_hdrs", "direct_srcs", "direct_object_files", "data"],
    ),
    "deps": attr.label_list(
        providers = [CcInfo, DefaultInfo],
        aspects = [extract_cc_srcs],
    ),
}

_do_extract_cc_srcs_hdrs_data = rule(
    implementation = _do_extract_impl,
    attrs = _attrs,
)
