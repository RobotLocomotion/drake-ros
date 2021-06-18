cc_library(
    name = "_" + @name@,
    srcs = @srcs@,
    hdrs = glob(["{}/**/*.*".format(x) for x in @headers@]),
    includes = @includes@,
    copts = @copts@,
    defines = @defines@,
    linkopts = @linkopts@,
    data = @data@,
    deps = @deps@,
)

dload_aware_cc_library(
    name = @name@,
    base = ":_" + @name@,
    data = @data@,
    deps = [":_" + @name@] + @deps@,
    env = @env@,
)
