#!/usr/bin/env python3

"""
Generates a runtime environment for RMW isolation.

Environment variables are written in the NAME=VALUE format,
one per line, suitable for e.g. the `env` command and the
`putenv` API.
"""

import argparse
import pathlib
import sys

from rmw_isolation import isolated_rmw_env

def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '-o', '--output', metavar='PATH',
        type=argparse.FileType('w'), default=sys.stdout,
        help='Path to output file. Defaults to stdout.')
    parser.add_argument(
        '-s', '--scratch-directory', metavar='PATH', default=pathlib.Path.cwd(),
        help=('Path to scratch directory for generated files, if any. '
              'Defaults to the current working directory.'))
    parser.add_argument(
        '-r', '--rmw-implementation', default=None,
        help=('Middleware implementation to be isolated. '
              'Defaults to currently applicable implementation, '
              'as returned by rclpy.get_rmw_implementation_identifier().'))
    parser.add_argument(
        'unique_identifier', nargs='?', default=str(pathlib.Path.cwd()),
        help=('Unique arbitrary identifier for isolation. '
              'Defaults to current working directory.'))
    args = parser.parse_args(argv)
    for key, value in isolated_rmw_env(
        args.unique_identifier,
        rmw_implementation=args.rmw_implementation,
        scratch_directory=args.scratch_directory
    ).items():
        args.output.write(f'{key}={value}\n')

if __name__ == '__main__':
    main()
