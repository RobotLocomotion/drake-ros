#!/usr/bin/env python3

import argparse
import glob
import os
import shutil
import sys

from tempfile import TemporaryDirectory
from textwrap import indent
import xml.etree.ElementTree as ET

import cmake_tools

from ros2bzl.scrapping.metadata import collect_package_metadata
from ros2bzl.scrapping.ament_cmake import collect_ament_cmake_package_direct_properties
from ros2bzl.scrapping.ament_python import collect_ament_python_package_direct_properties
from ros2bzl.templates import configure_package_share_filegroup
from ros2bzl.templates import configure_package_cc_library
from ros2bzl.templates import configure_package_py_library

from ros2bzl.resources import load_resource
from ros2bzl.resources import load_underlay
from ros2bzl.resources import path_to_resource

import ros2bzl.sandboxing as sandboxing

from ros2bzl.utilities import compose
from ros2bzl.utilities import interpolate


def generate_rosidl_interfaces(args):
    with TemporaryDirectory(dir=args.gen_directory) as tmpdir:
        local_interfaces = []
        for path in args.interfaces:
            symlink = os.path.join(
                tmpdir,
                os.path.basename(
                    os.path.dirname(path)
                ),
                os.path.basename(path)
            )
            os.makedirs(os.path.dirname(symlink), exist_ok=True)
            os.symlink(path, symlink)
            local_interfaces.append(
                os.path.relpath(symlink, start=tmpdir)
            )

        cmakelists_template_path = path_to_resource('rosidl_cmake_CMakeLists.txt.in')
        cmakelists_path = os.path.join(tmpdir, 'CMakeLists.txt')
        cmake_tools.configure_file(cmakelists_template_path, cmakelists_path, {
            '@NAME@': args.package_name,
            '@INTERFACES@': ' '.join(local_interfaces),
            '@DEPENDENCIES@': ' '.join(args.dependencies),
        })

        package_xml_template_path = path_to_resource('rosidl_package.xml.in')
        package_xml_path = os.path.join(tmpdir, 'package.xml')
        cmake_tools.configure_file(package_xml_template_path, package_xml_path, {
            '@NAME@': args.package_name, '@DEPENDENCIES@': '\n'.join([
                '<depend>' + dep + '</depend>' for dep in args.dependencies
            ]),
        })

        install_path = os.path.join(args.gen_directory, args.target_prefix)
        if os.path.exists(install_path):
            shutil.rmtree(install_path)
        os.makedirs(install_path)

        ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
        cmake_prefix_path = ament_prefix_path.replace(':', ';')
        cmake_tools.build_then_install(
            tmpdir,
            '-DCMAKE_PREFIX_PATH=' + cmake_prefix_path,
            '-DCMAKE_INSTALL_PREFIX=' + install_path,
        )

        return install_path


def generate_rosidl_file(args):
    packages, _, cache, sandbox = load_underlay()

    metadata = collect_package_metadata(
        args.package_name, prefix=generate_rosidl_interfaces(args)
    )

    noextras = None

    sandbox = compose(args.sandbox, sandbox)

    dependencies = {
        'rosidl_default_generators',
        'rosidl_default_runtime'
    }
    dependencies.update(metadata['build_dependencies'])
    dependencies.update(metadata['run_dependencies'])

    dependencies = {name: packages[name] for name in dependencies}

    args.output_file.write(load_resource('prologue.bzl') + '\n')

    metadata['bazel_name'] = args.target_prefix  # Use target prefix
    args.output_file.write('def {}():\n'.format(args.target_prefix))

    # Generate share files' group
    template, config = configure_package_share_filegroup(
        args.package_name, metadata, args.sandbox
    )
    args.output_file.write(
        indent(interpolate(template, config), ' ' * 4) + '\n'
    )

    # Generate C/C++ library
    properties = collect_ament_cmake_package_direct_properties(
        args.package_name, metadata, dependencies, cache
    )
    template, config = configure_package_cc_library(
        args.package_name, metadata, properties,
        dependencies, noextras, sandbox
    )
    args.output_file.write(
        indent(interpolate(template, config), ' ' * 4) + '\n'
    )

    # Generate Python package
    properties = collect_ament_python_package_direct_properties(
        args.package_name, metadata, dependencies, cache
    )
    template, config = configure_package_py_library(
        args.package_name, metadata, properties,
        dependencies, noextras, sandbox
    )
    args.output_file.write(indent(interpolate(template, config), ' ' * 4))


def name_from_package_xml():
    if not os.path.exists('package.xml'):
        return None
    tree = ET.parse('package.xml')
    return tree.find('./name').text


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-s', '--sandbox', action='append', default=[],
        help='Path mappings for sandboxing, in "outer:inner" format'
    )
    parser.add_argument(
        '-o', '--output-file', type=argparse.FileType('w'),
        default=sys.stdout, help='Output .bzl file (defaults to stdout)'
    )
    parser.add_argument(
        '--gen-directory', default='gen', help=(
            'Directory for generated files (relative to output file, if one is given)'
        )
    )
    parser.add_argument(
        '-d', '--dep', dest='dependencies', action='append', default=[],
        help='Packages from which interface types are used'
    )
    parser.add_argument(
        '-r', '--repository-name',
        default=os.environ.get('BAZEL_REPOSITORY_NAME', None),
        help='Bazel repository name'
    )
    parser.add_argument(
        '-p', '--package-name', default=name_from_package_xml(),
        help='Interfaces\' package name (defaults to package.xml content if any)'
    )
    parser.add_argument(
        '-t', '--target-prefix', default=None,
        help='Interface\' targets prefix (defaults to package name)'
    )
    parser.add_argument(
        'interfaces', nargs='+',
        help='Paths to interface files (may be glob patterns)'
    )
    args = parser.parse_args()

    if not args.repository_name:
        parser.error(
            'No repository name provided nor $BAZEL_REPOSITORY_NAME envvar found'
        )

    if not args.package_name:
        parser.error(
            'No package name provided nor package.xml found'
        )

    if args.output_file is not sys.stdout:
        output_dirpath = os.path.dirname(
            os.path.realpath(args.output_file.name)
        )
    else:
        output_dirpath = os.getcwd()
    os.makedirs(output_dirpath, exist_ok=True)

    args.gen_directory = os.path.realpath(args.gen_directory)
    if not args.gen_directory.startswith(output_dirpath):
        parser.error(
            'Generated files must be in subdirectories of the output file '
            'directory. All paths in a Bazel target must be relative.'
        )
    os.makedirs(args.gen_directory, exist_ok=True)

    args.sandbox.append(output_dirpath + ':.')
    args.sandbox = sandboxing.configure(
        name=args.repository_name, mapping=dict(
            entry.partition(':')[0::2] for entry in args.sandbox
        )
    )

    if not args.target_prefix:
        args.target_prefix = args.package_name

    args.interfaces = [
        os.path.realpath(path)
        for pattern in args.interfaces
        for path in glob.glob(pattern)
    ]

    return args


def main():
    generate_rosidl_file(parse_arguments())


if __name__ == '__main__':
    main()
