import importlib.metadata
import glob
import os.path
import sys
import sysconfig

from importlib.metadata import PackageNotFoundError

from ros2bzl.scraping.properties import PyProperties
from ros2bzl.scraping.system import find_library_dependencies
from ros2bzl.scraping.system import is_system_library

EXTENSION_SUFFIX = sysconfig.get_config_var('EXT_SUFFIX')


def find_package(name):
    dist = importlib.metadata.distribution(name)
    top_level = dist.read_text('top_level.txt')
    packages = top_level.splitlines()
    assert len(packages) > 0
    return str(dist._path), str(dist.locate_file(packages[0]))


def get_packages_with_prefixes(prefixes=None):
    packages = {}
    for dist in importlib.metadata.distributions():
        top_level = dist.read_text('top_level.txt')
        if top_level is None:
            continue
        top_level_path = dist.locate_file('top_level.txt')
        for package_name in top_level.splitlines():
            if prefixes is not None:
                for prefix in prefixes:
                    if top_level_path.is_relative_to(prefix):
                        packages[package_name] = prefix
                        break
            else:
                packages[package_name] = top_level_path.parent
    return packages


def collect_python_package_properties(name, metadata):
    properties = PyProperties()
    egg_path, top_level = find_package(name)
    properties.python_packages = tuple([(egg_path, top_level)])
    cc_libraries = glob.glob('{}/**/*.so'.format(top_level),  recursive=True)
    if cc_libraries:
        cc_libraries.extend(set(
            dep for library in cc_libraries
            for dep in find_library_dependencies(library)
            if not is_system_library(dep)
        ))
        properties.cc_extensions = [
            lib for lib in cc_libraries
            if lib.endswith(EXTENSION_SUFFIX)
        ]
        properties.cc_libraries = [
            lib for lib in cc_libraries
            if not lib.endswith(EXTENSION_SUFFIX)
        ]
    return properties
