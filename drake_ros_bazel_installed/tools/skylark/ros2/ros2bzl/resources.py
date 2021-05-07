import functools
import json
import os

import ros2bzl.sandboxing as sandboxing


PATH_TO_RESOURCES = os.path.realpath(
    os.path.join(os.path.dirname(__file__), '..', 'resources')
)


@functools.lru_cache(maxsize=None)
def path_to_resource(name):
    return os.path.join(PATH_TO_RESOURCES, name)


@functools.lru_cache(maxsize=None)
def load_resource(name):
    with open(path_to_resource(name), 'r') as fd:
        return fd.read()


ROS2BZL_PREFIX_PATH = os.environ.get('ROS2BZL_PREFIX_PATH', '.')


class ExtendedJSONEncoder(json.JSONEncoder):

    def default(self, o):
        try:
            iterable = iter(o)
        except TypeError:
            pass
        else:
            return list(iterable)
        return json.JSONEncoder.default(self, o)


def load_underlay():
    path = os.path.join(ROS2BZL_PREFIX_PATH, 'underlay.json')
    if not os.path.exists(path):
        raise ValueError(
            'No underlay to load: {} not found'.format(path)
        )
    with open(path, 'r') as fd:
        setup = json.load(fd)
    return (
        setup['packages'], setup['dependency_graph'],
        setup['cache'], sandboxing.configure(**setup['sandbox']),
    )


def setup_underlay(packages, dependency_graph, cache, sandbox):
    with open('underlay.json', 'w') as fd:
        json.dump({
            'packages': packages,
            'dependency_graph': dependency_graph,
            'cache': cache,
            'sandbox': sandbox.kwargs,
        }, fd, cls=ExtendedJSONEncoder, sort_keys=True, indent=4)
