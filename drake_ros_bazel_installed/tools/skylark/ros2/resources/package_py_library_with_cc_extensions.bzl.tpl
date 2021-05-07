drake_ros_py_library(
    name = {name},
    srcs = glob(["{{}}/**/*.py".format(x) for x in {packages}]),
    data = glob(
       include=["{{}}/**/*.*".format(x) for x in {packages}],
       exclude=["**/*.py", "**/*.so"],
    ) + {data},
    imports = {imports},
    deps = {deps},
    runenv = {runenv},
)
