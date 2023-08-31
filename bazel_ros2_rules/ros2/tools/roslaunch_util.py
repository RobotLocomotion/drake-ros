import subprocess
import sys
import os

from launch.actions import ExecuteProcess
from launch.frontend import expose_action

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

    # TODO : Implement this
    @classmethod
    def parse(cls, entity, parser):
        pass

    def execute(self, context):
        return super().execute(context)

if __name__ == '__main__':
    launch_file_name = sys.argv[1]

    roslaunch_cli = "./external/ros2/ros2"
    action = "launch"
    # TODO : Is there a better way to locate the launch file exactly ?
    launch_file = find_rel_path(launch_file_name, os.getcwd())

    env = {**os.environ, 'PYTHONPATH': os.getcwd() + '/external/bazel_ros2_rules/ros2/tools:'
           + os.environ['PYTHONPATH']}
    subprocess.run([roslaunch_cli, action, launch_file], env=env)
    # TODO (Aditya): For debugging, to be removed
    # subprocess.run(["/bin/bash"], env=env)
