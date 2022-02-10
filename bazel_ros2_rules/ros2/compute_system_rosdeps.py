#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys

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
        'distro_file', type=argparse.FileType('r'),
        help='Path to distro metadata file, in JSON format'
    )
    args = parser.parse_args()

    args.distro = json.load(args.distro_file)

    return args


# The list of rosdep keys that are skipped has been taken verbatim from
# ROS 2 Rolling binary install docs.
#
# This is necessary because:
# - Some non-ROS packages don't always install their package manifests
#   (cyclonedds, fastcdr, fastrtps, urdfdom_headers)
# - Group dependencies aren't supported everywhere and are hard-coded in
#   some packages (rti-connext-dds-5.3.1)
#
# See https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html
# for further reference.
SKIPPED_ROSDEP_KEYS = {
    'cyclonedds', 'fastcdr', 'fastrtps',
    'rti-connext-dds-5.3.1', 'urdfdom_headers'}

def compute_system_rosdeps(distro):
    cmd = [
        'rosdep', 'keys', '-i',
        '-t', 'buildtool_export',
        '-t', 'build_export',
        '-t', 'exec', '--from-paths'
    ] + distro['paths']['ament_prefix']
    output = subprocess.check_output(
        cmd, env={'ROS_PYTHON_VERSION': '3'},
        encoding='utf-8')
    rosdep_keys = set(output.splitlines())
    return sorted(rosdep_keys - SKIPPED_ROSDEP_KEYS)


def main():
    args = parse_arguments()

    args.output.write('\n'.join(
        compute_system_rosdeps(args.distro)) + '\n')


if __name__ == '__main__':
    main()
