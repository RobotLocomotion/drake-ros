"""
Utility functions to aid compilation and linkage configuration scraping by
resorting to (Linux) system-wide conventions and tooling.
"""

import os
import re
import subprocess
import sys


# Standard include files' search paths for compilers in Linux systems.
# Useful to detect system includes in package exported configuration.
DEFAULT_INCLUDE_DIRECTORIES = ['/usr/include', '/usr/local/include']


def is_system_include(include_path):
    """
    Checks whether `include_path` is in a system include directory
    i.e. known to compilers.
    """
    include_path = os.path.realpath(include_path)
    return any(
        include_path.startswith(path)
        for path in DEFAULT_INCLUDE_DIRECTORIES)


# Standard library files' search paths for linkers in Linux systems.
# Useful to detect system libraries in package exported configuration.
DEFAULT_LINK_DIRECTORIES = ['/lib', '/usr/lib', '/usr/local/lib']


def is_system_library(library_path):
    """
    Checks whether `library_path` is in a system library directory
    i.e. known to linkers.
    """
    library_path = os.path.realpath(library_path)
    return any(
        library_path.startswith(path)
        for path in DEFAULT_LINK_DIRECTORIES)


LD_LIBRARY_PATHS = [
    path for path in os.environ.get('LD_LIBRARY_PATH', '').split(':') if path]


def find_library_path(library_name, link_directories=None, link_flags=None):
    """
    Finds the path to a library.

    To do so it relies on the GNU `ld` linker and its library naming spec,
    effectively reusing all of its search logic.

    :param library_name: name of the library to be searched
    :param link_directories: optional list of directories to search in
    :param link_flags: optional list of linker flags to account for
    :returns: the path to the library if found, None otherwise
    """
    # Adapted from ctypes.util.find_library_path() implementation.
    pattern = r'/(?:[^\(\)\s]*/)*lib{}\.[^\(\)\s]*'.format(library_name)

    cmd = ['ld', '-t']
    if link_directories:
        for path in link_directories:
            cmd.extend(['-L', path])
    if link_flags:
        cmd.extend(link_flags)
    for path in LD_LIBRARY_PATHS:
        cmd.extend(['-L', path])
    for path in DEFAULT_LINK_DIRECTORIES:
        cmd.extend(['-L', path])
    cmd.extend(['-o', os.devnull, '-l' + library_name])
    try:
        out = subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            encoding='utf8'
        ).stdout.strip()
        match = re.search(pattern, out)
        if match:
            path = match.group(0)
            # Remove double forward slashes, if any
            return os.path.join('/', os.path.relpath(path, '/'))
    except Exception:
        pass
    return None


LDD_LINE_PATTERN = re.compile(r' => (/(?:[^\(\)\s]*/)*lib[^\(\)\s]*)')


def find_library_dependencies(library_path):
    """
    Lists all shared library dependencies of a given library.

    To do so, it relies on the `ldd` tool as found in Linux.

    :param library_path: path to the library to be inspected
    :returns: a generator that iterates over the paths to
      library dependencies
    """
    try:
        lines = subprocess.run(
            ['ldd', library_path],
            check=True,
            stdout=subprocess.PIPE,
            encoding='utf8'
        ).stdout.strip().split('\n')
        for line in lines:
            match = LDD_LINE_PATTERN.search(line.strip())
            if match:
                yield match.group(1)
    except Exception:
        pass
    return
