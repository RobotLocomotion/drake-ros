#!/usr/bin/env python3

import argparse
import collections
import os
import sys

from multiprocessing.dummy import Pool
import xml.etree.ElementTree as ET

import toposort

sys.path.insert(0, os.path.dirname(__file__))  # noqa

from ros2bzl.scrapping import index_all_packages
from ros2bzl.scrapping import build_dependency_graph
from ros2bzl.scrapping.ament_cmake import collect_ament_cmake_package_properties
from ros2bzl.scrapping.ament_cmake import collect_ament_cmake_package_direct_properties
from ros2bzl.scrapping.ament_python import collect_ament_python_package_direct_properties
from ros2bzl.scrapping.ament_python import PackageNotFoundError

from ros2bzl.templates import configure_package_meta_py_library
from ros2bzl.templates import configure_package_alias
from ros2bzl.templates import configure_package_cc_library
from ros2bzl.templates import configure_package_executable_imports
from ros2bzl.templates import configure_package_py_library
from ros2bzl.templates import configure_package_share_filegroup

from ros2bzl.resources import load_resource
from ros2bzl.resources import setup_underlay

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


def generate_build_file(packages, dependency_graph, cache, extras, sandbox):
    rmw_implementation_packages = {
        name: metadata for name, metadata in packages.items()
        if 'rmw_implementation_packages' in metadata['groups']
    }

    with open('BUILD.bazel', 'w') as fd:
        fd.write(load_resource('BUILD.prologue.bazel') + '\n')

        for name in toposort.toposort_flatten(dependency_graph):
            metadata = packages[name]

            dependencies = {
                dependency_name: packages[dependency_name]
                for dependency_name in dependency_graph[name]
            }

            template, config = \
                configure_package_share_filegroup(name, metadata, sandbox)
            fd.write(interpolate(template, config) + '\n')

            main_target = None
            if metadata['build_type'] == 'ament_cmake':
                properties = collect_ament_cmake_package_direct_properties(
                    name, metadata, dependencies, cache
                )

                template, config = configure_package_cc_library(
                    name, metadata, properties, dependencies, extras, sandbox
                )

                if any(properties.values()):
                    # Tentatively set as main target (for aliasing)
                    main_target = config['name']

                fd.write(interpolate(template, config) + '\n')

            # No way to tell if there's Python code for this package
            # but to look for it.
            try:
                properties = collect_ament_python_package_direct_properties(
                    name, metadata, dependencies, cache
                )
                # Add 'py' as language if not there.
                metadata['langs'].add('py')
            except PackageNotFoundError:
                if any('py' in metadata['langs'] for metadata in dependencies.values()):
                    metadata['langs'].add('py (transitively)')
                    # Dependencies still need to be propagated.
                    template, config = \
                        configure_package_meta_py_library(name, metadata, dependencies)
                    fd.write(interpolate(template, config) + '\n')

                properties = {}

            if properties:
                template, config = configure_package_py_library(
                    name, metadata, properties, dependencies, extras, sandbox
                )

                # Set as main target if not set already,
                # otherwise there's no main target.
                main_target = config['name'] if not main_target else None

                fd.write(interpolate(template, config) + '\n')

            if main_target is not None and main_target != name:
                template, config = configure_package_alias(name, main_target)
                fd.write(interpolate(template, config) + '\n')

            dependencies.update(rmw_implementation_packages)
            for template, config in configure_package_executable_imports(
                name, metadata, dependencies, extras, sandbox
            ):
                fd.write(interpolate(template, config) + '\n')


def precache_ament_cmake_properties(packages, jobs=None):
    ament_cmake_packages = {
        name: metadata
        for name, metadata in packages.items()
        if metadata['build_type'] == 'ament_cmake'
    }
    with Pool(jobs) as pool:
         return dict(zip(
            ament_cmake_packages.keys(), pool.starmap(
                collect_ament_cmake_package_properties,
                ament_cmake_packages.items()
            )
        ))


def main():
    args = parse_arguments()

    packages, dependency_graph = build_dependency_graph(
        index_all_packages(),
        set(args.include_packages),
        set(args.exclude_packages)
    )

    cache = {
        'ament_cmake': precache_ament_cmake_properties(packages, jobs=args.jobs)
    }

    generate_build_file(packages, dependency_graph, cache, args.extras, args.sandbox)

    for name, metadata in packages.items():
        # For downstream repositories to use
        metadata['bazel_workspace'] = args.repository_name
    setup_underlay(packages, dependency_graph, cache, args.sandbox)


if __name__ == '__main__':
    main()
