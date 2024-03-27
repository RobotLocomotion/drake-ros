import subprocess
import sys
import os

from launch.actions import ExecuteProcess
from launch.frontend import expose_action

# This util file serves as a wrapper over the cmdline
# way to run launch, using "ros2 launch launch_file.py".
# The ros_launch() bazel rule starts this script, which
# gets run as a ros_py_binary(). This way we get the ros
# environment set up for us for free.

def find_rel_path(name, path):
    for root, _, files in os.walk(path):
        if name in files:
            return os.path.relpath(os.path.join(root, name))

# Launch action to wrap over ExecuteProcess.
@expose_action('execute_bazel_target')
class ExecuteBazelTarget(ExecuteProcess):
    def __init__(self,target,**kwargs):
        super().__init__(cmd=[find_rel_path(target, os.getcwd())],
                         **kwargs)

    @classmethod
    def parse(cls, entity, parser):
        _, kwargs = super().parse(entity, parser, ignore=['cmd'])
        kwargs['target'] = entity.get_attr('target')
        return cls, kwargs

    def execute(self, context):
        return super().execute(context)

if __name__ == '__main__':
    # TODO (Aditya): How do I stop installing this every time ?
    # Required to get xml launch working, as we need to register an entry
    # point for launch extensions.
    os.system("pip install ../bazel_ros2_rules/ros2/tools/roslaunch_util >/dev/null 2>&1")
    # Actual launch files should be able to import the custom action now.
    env = {**os.environ, 'PYTHONPATH': os.getcwd() + '/external/bazel_ros2_rules/ros2/tools:'
           + os.environ['PYTHONPATH']}

    launch_file_name = sys.argv[1]

    roslaunch_cli = "./external/ros2/ros2"
    action = "launch"
    # TODO : Is there a better way to locate the launch file exactly ?
    launch_file = find_rel_path(launch_file_name, os.getcwd())

    # This basically runs "ros2 launch launch_file"
    subprocess.run([roslaunch_cli, action, launch_file], env=env)
