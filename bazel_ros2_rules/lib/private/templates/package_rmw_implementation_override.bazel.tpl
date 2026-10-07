genrule(
    name = @name@,
    srcs = @srcs@,
    outs = @outs@,
    cmd = "cp $< $@",
    visibility = ["//visibility:public"],
)
