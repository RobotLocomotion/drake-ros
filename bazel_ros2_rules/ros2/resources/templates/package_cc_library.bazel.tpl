cc_library(
    name = @name@,
    srcs = @srcs@,
    hdrs = glob(["{}/**/*.h*".format(x) for x in @headers@], allow_empty = True),
    includes = @includes@,
    copts = @copts@,
    defines = @defines@,
    linkopts = @linkopts@,
    data = @data@,
    deps = @deps@,
)
