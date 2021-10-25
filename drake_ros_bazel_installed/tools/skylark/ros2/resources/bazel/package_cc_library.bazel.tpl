cc_library(
    name = @name@,
    srcs = @srcs@,
    hdrs = glob(["{}/**/*.*".format(x) for x in @headers@]),
    includes = @includes@,
    copts = @copts@,
    defines = @defines@,
    linkopts = @linkopts@,
    data = @data@,
    deps = @deps@,
)
