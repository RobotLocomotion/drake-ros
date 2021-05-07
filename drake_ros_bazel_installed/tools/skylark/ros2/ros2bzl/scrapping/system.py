import os
import re
import subprocess
import sys

DEFAULT_INCLUDE_DIRECTORIES = ['/usr/include', '/usr/local/include']

def is_system_include(include_path):
    include_path = os.path.realpath(include_path)
    return any(include_path.startswith(path) for path in DEFAULT_INCLUDE_DIRECTORIES)


DEFAULT_LINK_DIRECTORIES = ['/lib', '/usr/lib', '/usr/local/lib']


def is_system_library(library_path):
    library_path = os.path.realpath(library_path)
    return any(library_path.startswith(path) for path in DEFAULT_LINK_DIRECTORIES)


LD_LIBRARY_PATHS = []
if 'LD_LIBRARY_PATH' in os.environ:
    value = os.environ['LD_LIBRARY_PATH']
    LD_LIBRARY_PATHS = [path for path in value.split(':') if path]


def find_library_path(library_name, link_directories=[], link_flags=[]):
    # Adapted from ctypes.util.find_library_path() implementation.
    pattern = r'/(?:[^\(\)\s]*/)*lib{}\.[^\(\)\s]*'.format(library_name)

    cmd = ['ld', '-t']
    for path in link_directories:
        cmd.extend(['-L', path])
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
    except Exception as e:
        pass
    return None


LDD_LINE_PATTERN = re.compile(r' => (/(?:[^\(\)\s]*/)*lib[^\(\)\s]*)')


def find_library_dependencies(library_path):
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
        return


def find_python_package(name, sysroot='/usr'):
    v = sys.version_info
    path = '{}/lib/python{}.{}/site-packages/{}'.format(
        sysroot, v.major, v.minor, name
    )
    if not os.path.exists(os.path.join(path, '__init__.py')):
        raise ValueError(name + ' is not a Python package')
    return path
