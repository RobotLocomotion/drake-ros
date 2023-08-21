from setuptools import setup

setup(
    name = "roslaunch_util",
    entry_points={
        'launch.frontend.launch_extension': [
            'roslaunch_util = roslaunch_util',
        ],
    }
)
