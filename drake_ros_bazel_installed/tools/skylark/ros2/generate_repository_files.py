#!/usr/bin/env python3

import argparse
import collections
import os
import sys

import xml.etree.ElementTree as ET

import toposort

sys.path.insert(0, os.path.dirname(__file__))  # noqa

from ros2bzl.scrapping import load_distribution
from ros2bzl.scrapping.ament_cmake import collect_ament_cmake_package_properties
from ros2bzl.scrapping.ament_cmake import collect_ament_cmake_package_direct_properties
from ros2bzl.scrapping.ament_cmake import precache_ament_cmake_properties
from ros2bzl.scrapping.ament_python import collect_ament_python_package_direct_properties
from ros2bzl.scrapping.ament_python import PackageNotFoundError

from ros2bzl.templates import configure_cc_tools
from ros2bzl.templates import configure_distro
from ros2bzl.templates import configure_executable_imports
from ros2bzl.templates import configure_package_meta_py_library
from ros2bzl.templates import configure_package_alias
from ros2bzl.templates import configure_package_c_library_alias
from ros2bzl.templates import configure_package_cc_library
from ros2bzl.templates import configure_package_executable_imports
from ros2bzl.templates import configure_package_py_library
from ros2bzl.templates import configure_package_share_filegroup
from ros2bzl.templates import configure_package_interfaces_filegroup
from ros2bzl.templates import configure_py_tools
from ros2bzl.templates import configure_prologue
from ros2bzl.templates import configure_rosidl_tools

from ros2bzl.resources import load_resource

import ros2bzl.sandboxing as sandboxing

from ros2bzl.utilities import interpolate


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'repository_name', help='Bazel repository name'
    )
    parser.add_argument(
        '-s', '--sandbox', action='append', default=[],
        help='Path mappings for sandboxing, in "outer:inner" format'
    )
    parser.add_argument(
        '-i', '--include-package', action='append', dest='include_packages', default=[],
        help='Packages to be included (plus their dependencies)'
    )
    parser.add_argument(
        '-e', '--exclude-package', action='append', dest='exclude_packages',
        default=[], help='Packages to be explicitly excluded'
    )
    parser.add_argument(
        '-x', '--extras', action='append', default=[],
        help=('Additional dependencies for generated targets,'
              ' in "label.attribute+=label_or_string" format')
    )
    parser.add_argument(
        '-j', '--jobs', type=int, default=None, help='Number of jobs to use'
    )
    args = parser.parse_args()

    extras = {}
    for item in args.extras:
        lhs, _, rhs = item.partition('+=')
        target_name, _, attribute_name = lhs.rpartition('.')
        if attribute_name not in extras:
            extras[attribute_name] = {}
        if target_name not in extras[attribute_name]:
            extras[attribute_name][target_name] = []
        extras[attribute_name][target_name].append(rhs)
    args.extras = extras

    args.sandbox = sandboxing.configure(
        name=args.repository_name, mapping=dict(
            entry.partition(':')[0::2] for entry in args.sandbox
        )
    )

    return args


def generate_distro_file(packages):
    with open('distro.bzl', 'w') as fd:
        template, config = configure_distro(packages)
        fd.write(interpolate(template, config) + '\n')


def generate_cc_tools_file(repo_name):
    with open('cc_tools.bzl', 'w') as fd:
        template, config = configure_cc_tools(repo_name)
        fd.write(interpolate(template, config) + '\n')


def generate_py_tools_file(repo_name):
    with open('py_tools.bzl', 'w') as fd:
        template, config = configure_py_tools(repo_name)
        fd.write(interpolate(template, config) + '\n')


def generate_rosidl_tools_file(repo_name):
    with open('rosidl_tools.bzl', 'w') as fd:
        template, config = configure_rosidl_tools(repo_name)
        fd.write(interpolate(template, config) + '\n')


def generate_build_file(repo_name, distro, cache, extras, sandbox):
    rmw_implementation_packages = {
        name: metadata for name, metadata in distro['packages'].items()
        if 'rmw_implementation_packages' in metadata.get('groups', [])
    }

    with open('BUILD.bazel', 'w') as fd:
        template, config = configure_prologue(repo_name)
        fd.write(interpolate(template, config) + '\n')

        for name in toposort.toposort_flatten(distro['dependency_graph']):
            metadata = distro['packages'][name]

            dependencies = {
                dependency_name: distro['packages'][dependency_name]
                for dependency_name in distro['dependency_graph'][name]
            }

            targets = []

            if 'share_directory' in metadata:
                _, template, config = \
                    configure_package_share_filegroup(name, metadata, sandbox)
                fd.write(interpolate(template, config) + '\n')

            if 'rosidl_interface_packages' in metadata.get('groups', []):
                label, template, config = \
                    configure_package_interfaces_filegroup(name, metadata, sandbox)
                fd.write(interpolate(template, config) + '\n')
                targets.append(label)

            if 'cmake' in metadata.get('build_type'):
                properties = collect_ament_cmake_package_direct_properties(
                    name, metadata, dependencies, cache
                )

                label, template, config = configure_package_cc_library(
                    name, metadata, properties, dependencies, extras, sandbox
                )

                if any(properties.values()):
                    targets.append(label)

                fd.write(interpolate(template, config) + '\n')

                if 'rosidl_interface_packages' in metadata.get('groups', []):
                    # Alias C++ library as C library for interface packages
                    # as their headers and artifacts cannot be discriminated.
                    _, template, config = \
                        configure_package_c_library_alias(name, metadata)
                    fd.write(interpolate(template, config) + '\n')

            # No way to tell if there's Python code for this package
            # but to look for it.
            try:
                properties = collect_ament_python_package_direct_properties(
                    name, metadata, dependencies, cache
                )
                # Add 'py' as language if not there.
                if 'langs' not in metadata:
                    metadata['langs'] = set()
                metadata['langs'].add('py')
            except PackageNotFoundError:
                if any('py' in metadata.get('langs', []) for metadata in dependencies.values()):
                    metadata['langs'].add('py (transitively)')
                    # Dependencies still need to be propagated.
                    _, template, config = \
                        configure_package_meta_py_library(name, metadata, dependencies)
                    fd.write(interpolate(template, config) + '\n')

                properties = {}

            if properties:
                label, template, config = configure_package_py_library(
                    name, metadata, properties, dependencies, extras, sandbox
                )
                fd.write(interpolate(template, config) + '\n')
                targets.append(label)

            if len(targets) == 1 and targets[0] != name:
                _, template, config = configure_package_alias(name, targets[0])
                fd.write(interpolate(template, config) + '\n')

            if metadata.get('executables'):
                dependencies.update(rmw_implementation_packages)
                for _, template, config in configure_package_executable_imports(
                    name, metadata, dependencies, sandbox, extras=extras
                ):
                    fd.write(interpolate(template, config) + '\n')

        for _, template, config in configure_executable_imports(
            distro['executables'], distro['packages'], sandbox, extras=extras
        ):
            fd.write(interpolate(template, config) + '\n')


def main():
    args = parse_arguments()

    distro = load_distribution(
        args.sandbox,
        set(args.include_packages),
        set(args.exclude_packages))

    cache = {
        'ament_cmake': precache_ament_cmake_properties(
            distro['packages'], jobs=args.jobs
        )
    }

    generate_build_file(
        args.repository_name, distro,
        cache, args.extras, args.sandbox)

    generate_distro_file(distro)

    generate_cc_tools_file(args.repository_name)

    generate_py_tools_file(args.repository_name)

    generate_rosidl_tools_file(args.repository_name)


if __name__ == '__main__':
    main()
