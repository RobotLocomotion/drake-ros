#!/usr/bin/env python3

import argparse
import json
import pathlib
import subprocess
import sys


# The list of rosdep keys that are skipped has been taken verbatim from
# ROS 2 Rolling binary install docs.
#
# This is necessary because:
# - Some non-ROS packages don't always install their package manifests
#   (cyclonedds, fastcdr, fastrtps, iceoryx_binding_c, urdfdom_headers)
# - Group dependencies aren't supported everywhere and are hard-coded in
#   some packages (rti-connext-dds-5.3.1)
#
# See https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html
# for further reference.
SKIPPED_ROSDEP_KEYS = {
    'cyclonedds', 'fastcdr', 'fastrtps', 'iceoryx_binding_c',
    'rti-connext-dds-5.3.1', 'urdfdom_headers',
    'rosidl_typesupport_fastrtps_cpp', 'rosidl_typesupport_fastrtps_c'}


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        '-o', '--output',
        type=argparse.FileType('w'), default=sys.stdout,
        help='Path to file to write BUILD.bazel content to'
    )
    parser.add_argument(
        'distro_file',
        type=argparse.FileType('r'),
        help='Path to distro metadata file, in JSON format'
    )
    args = parser.parse_args()
    args.distro = json.load(args.distro_file)

    return args


def compute_system_rosdeps(distro):
    cmd = [
        'rosdep', 'keys', '-i',
        '-t', 'buildtool_export',
        '-t', 'build_export',
        '-t', 'exec', '--from-paths'
    ] + [
        metadata['share_directory']
        for metadata in distro['packages'].values()
        if 'share_directory' in metadata
    ]
    output = subprocess.check_output(
        cmd, env={'ROS_PYTHON_VERSION': '3'},
        encoding='utf-8')
    rosdep_keys = set(output.splitlines())
    return sorted(rosdep_keys - SKIPPED_ROSDEP_KEYS)


def main():
    args = parse_arguments()

    system_rosdeps = compute_system_rosdeps(args.distro)
    args.output.write('\n'.join(system_rosdeps) + '\n')


if __name__ == '__main__':
    main()
