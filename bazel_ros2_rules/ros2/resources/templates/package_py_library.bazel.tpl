py_library(
    name = @name@,
    srcs = glob(["{}/**/*.py".format(x) for x in @tops@], allow_empty = True),
    data = glob(
       include=[
         "{}/**/*.*".format(x) for x in @tops@
       ] + [
         "{}/*".format(x) for x in @eggs@
       ],
       exclude=["**/*.py", "**/*.so"],
       allow_empty = True,
    ) + @data@,
    imports = @imports@,
    deps = @deps@,
)
