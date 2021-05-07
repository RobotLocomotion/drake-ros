from ament_index_python import get_packages_with_prefixes

from ros2bzl.scrapping.metadata import collect_package_metadata


def index_all_packages():
    return {
        name: collect_package_metadata(name, prefix)
        for name, prefix in get_packages_with_prefixes().items()
    }



def build_dependency_graph(packages, include=None, exclude=None):
    package_set = set(packages)
    if include:
        package_set &= include
    if exclude:
        package_set -= exclude

    dependency_graph = {}
    while package_set:
        name = package_set.pop()
        metadata = packages[name]
        dependencies = set(metadata['build_dependencies'])
        dependencies.update(metadata['run_dependencies'])
        if exclude:
            dependencies -= exclude
        # Ignore system, non-ROS dependencies
        # NOTE(hidmic): shall we sandbox those too?
        dependencies = {
            dependency_name
            for dependency_name in dependencies
            if dependency_name in packages
        }
        dependency_graph[name] = dependencies
        package_set.update(dependencies)

    packages = {name: packages[name] for name in dependency_graph}

    return packages, dependency_graph
