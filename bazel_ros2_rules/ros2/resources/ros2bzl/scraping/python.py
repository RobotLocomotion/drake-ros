import importlib.metadata
import glob
import os.path
import sys
import sysconfig

from collections.abc import Sequence
from typing import Any, Dict, Final, Optional, Tuple

from importlib.metadata import PackageNotFoundError

from ros2bzl.scraping.properties import PyProperties
from ros2bzl.scraping.system import find_library_dependencies
from ros2bzl.scraping.system import is_system_library

EXTENSION_SUFFIX: Final[str] = sysconfig.get_config_var('EXT_SUFFIX')


def find_package(name: str) -> Tuple[str, str]:
    """Find a Python package path and top level module path given its `name`."""
    dist = importlib.metadata.distribution(name)
    top_level = dist.read_text('top_level.txt')
    packages = top_level.splitlines()
    assert len(packages) == 1
    return str(dist._path), str(dist.locate_file(packages[0]))


def get_packages_with_prefixes(prefixes: Optional[Sequence[str]] = None) -> Dict[str, pathlib.Path]:
    """
    Get all importable Python packages and the prefixes under which these can be found.

    If no `prefixes` are given, the entire `sys.path` is used.
    """
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


def collect_python_package_properties(name: str, metadata: Dict[str, Any]) -> PyProperties:
    """Collect Python library properties given package `name` and `metadata`."""
    properties = PyProperties()
    egg_path, top_level = find_package(name)
    properties.python_packages = tuple([(egg_path, top_level)])
    cc_libraries = glob.glob('{}/**/*.so'.format(top_level), recursive=True)
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
