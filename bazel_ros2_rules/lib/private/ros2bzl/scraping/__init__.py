import copy
import os

from cmake_tools import get_packages_with_prefixes

from ros2bzl.scraping.metadata import collect_cmake_package_metadata
from ros2bzl.scraping.metadata import collect_python_package_metadata
from ros2bzl.scraping.metadata import collect_ros_package_metadata

from ros2bzl.scraping.python import (
    get_packages_with_prefixes as get_python_packages_with_prefixes
)

from ros2bzl.utilities import ordered_set


def list_all_executables():
    # Delay import to allow testing most of ros2bzl without a ros2 workspace
    import ament_index_python
    executables = {}
    for prefix in ament_index_python.get_packages_with_prefixes().values():
        bindir = os.path.join(prefix, 'bin')
        if not os.path.isdir(bindir):
            continue
        for path in os.listdir(bindir):
            path = os.path.join(bindir, path)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                name = os.path.basename(path)
                if name not in executables:
                    executables[name] = path
    return list(executables.values())


def index_all_packages():
    # Delay import to allow testing most of ros2bzl without a ros2 workspace
    import ament_index_python
    packages = {
        name: collect_ros_package_metadata(name, prefix)
        for name, prefix in
        ament_index_python.get_packages_with_prefixes().items()
    }
    search_paths = ament_index_python.get_search_paths()
    cmake_packages = get_packages_with_prefixes(search_paths)
    for name, prefix in cmake_packages.items():
        if name in packages:
            # Assume unique package names across package types
            continue
        packages[name] = collect_cmake_package_metadata(name, prefix)
    python_packages = get_python_packages_with_prefixes(search_paths)
    for name, prefix in python_packages.items():
        if name in packages:
            # Assume unique package names across package types
            continue
        packages[name] = collect_python_package_metadata(name, prefix)
    return packages


def build_dependency_graph(packages, include=None, exclude=None):
    package_set = ordered_set(packages)
    if include:
        include = ordered_set(include)
        if not all(p in package_set for p in include):
            unknown_packages = [p for p in include if p not in package_set]
            msg = 'Cannot find package'
            if len(unknown_packages) == 1:
                msg += ' ' + repr(unknown_packages[0])
            else:
                msg += 's ' + repr(unknown_packages)
            raise RuntimeError(msg)
        package_set = include
    if exclude:
        package_set = [p for p in package_set if p not in exclude]

    groups = {}
    for name, metadata in packages.items():
        if 'groups' not in metadata:
            continue
        for group_name in metadata['groups']:
            groups.setdefault(group_name, [])
            groups[group_name].append(name)

    dependency_graph = {}
    while package_set:
        name = package_set.pop()
        metadata = packages[name]
        dependencies = copy.deepcopy(metadata.get('build_export_dependencies', []))
        dependencies += [dep for dep in metadata.get('run_dependencies', []) if dep not in dependencies]
        if 'group_dependencies' in metadata:
            for group_name in metadata['group_dependencies']:
                dependencies += [dep for dep in groups[group_name] if dep not in dependencies]
        if exclude:
            dependencies = [dep for dep in dependencies if dep not in exclude]
        # Ignore system, non-ROS dependencies
        # NOTE(hidmic): shall we sandbox those too?
        dependencies = ordered_set([
            dependency_name
            for dependency_name in dependencies
            if dependency_name in packages
        ])
        dependency_graph[name] = dependencies
        package_set += [d for d in dependencies if d not in package_set and d not in dependency_graph]

    packages = {name: packages[name] for name in dependency_graph}

    return packages, dependency_graph


def scrape_distribution(include=None, exclude=None):
    # Delay import to allow testing most of ros2bzl without a ros2 workspace
    import ament_index_python
    packages, dependency_graph = build_dependency_graph(
        index_all_packages(), include, exclude)
    executables = list_all_executables()
    ld_library_path = os.environ['LD_LIBRARY_PATH']
    ros_distro = os.environ['ROS_DISTRO']
    return {
        'packages': packages,
        'dependency_graph': dependency_graph,
        'executables': executables,
        'ros_distro': ros_distro,
        'paths': {
            'ament_prefix': ament_index_python.get_search_paths(),
            'library_load': ld_library_path.split(os.path.pathsep),
        }
    }
