load("@ros2//:ros_py.bzl", "ros_py_binary")

def ros_launch(name, launch_file):
    """Creates a Python script that calls the ROS2 launch API with a given launch file.

    Args:
        name: target name for the executable launch
        launch_file: file path to the yaml or xml launch file.
    """

    # Based on ros launch unit test usage here
    # https://github.com/ros2/launch/blob/f70f90111451b2f4d866bf3a3092afd36f8dc26f/launch_yaml/test/launch_yaml/test_executable.py#L24
    native.genrule(
        name = "{}_gen_launch_script".format(name),
        outs = ["{}_launch_script.py".format(name)],
        cmd = "sed 's/{{}}/{lf}/g' $(location @bazel_ros2_rules//ros2:ros_launch_template) > $@".format(
            lf = launch_file,
        ),
        executable = True,
        srcs = [launch_file, "@bazel_ros2_rules//ros2:ros_launch_template"],
    )

    ros_py_binary(
        name = "{}".format(name),
        srcs = ["{}_launch_script.py".format(name)],
        main = "{}_launch_script.py".format(name),
        data = [launch_file],
        deps = [
            "@ros2//:launch_xml_py",
            "@ros2//:launch_yaml_py",
        ],
    )
