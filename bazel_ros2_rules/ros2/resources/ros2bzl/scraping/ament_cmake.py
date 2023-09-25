import glob
from multiprocessing.dummy import Pool
import os
from pathlib import Path
import re
from tempfile import TemporaryDirectory

import cmake_tools
from ros2bzl.resources import path_to_resource
from ros2bzl.scraping.properties import CcProperties
from ros2bzl.scraping.system import find_library_path
from ros2bzl.scraping.system import find_library_dependencies
from ros2bzl.scraping.system import is_system_include
from ros2bzl.scraping.system import is_system_library


_ALLOWED_SYSTEM_LIBS = [
    # Allow console_bridge_vendor to use upstream package on Ubuntu Jammy
    re.compile(r"libconsole_bridge\.so[.0-9]*"),
]


def collect_ament_cmake_shared_library_codemodel(
    target, additional_libraries
) -> CcProperties:
    assert 'SHARED_LIBRARY' == target.target_type

    link_flags = []
    for flag in target.link_flags:
        flag = flag.strip()
        if not flag:
            continue
        link_flags.append(flag)

    link_libraries = []
    libraries = []
    for item in target.link_libraries:
        item = item.strip()
        if item.startswith('-Wl,-rpath'):
            # Ignore rpath setting flags
            continue
        if item.startswith('-') and not item.startswith('-l'):
            link_flags.append(item)
            continue
        libraries.append(item)
    for library in additional_libraries:
        if library in libraries:
            continue
        libraries.append(library)
    local_link_libraries = []
    for library in libraries:
        if not os.path.isabs(library):
            if library.startswith('-l'):
                library = library[2:]
            library = find_library_path(
                library,
                # Use linker options as well
                link_flags=link_flags
            )
            if not library:
                # Ignore and keep going
                continue
        # Some packages do not fully export runtime dependencies.
        # NOTE(hidmic): can the CMake registry be used instead
        # of the ament index to mitigate this?
        library_plus_dependencies = [library]
        library_plus_dependencies += list(
            find_library_dependencies(library)
        )
        # Remove duplicates maintaining order.
        for library in library_plus_dependencies:
            if library in link_libraries:
                continue
            if is_system_library(library):
                if library.startswith('/usr/local'):
                    local_link_libraries.append(library)
                for allowed_regex in _ALLOWED_SYSTEM_LIBS:
                    if allowed_regex.fullmatch(os.path.basename(library)):
                        link_flags.append('-L' + os.path.dirname(library))
                        link_flags.append('-l:' + os.path.basename(library))
                        break
                continue
            link_libraries.append(library)
    # Fail on any /usr/local libraries
    if local_link_libraries:
        error_message = 'Found libraries under /usr/local: '
        error_message += ', '.join(local_link_libraries)
        raise RuntimeError(error_message)

    local_include_directories = []
    include_directories = []
    for path in target.includes + target.system_includes:
        include_directories.append(path)
        if is_system_include(path):
            if path.startswith('/usr/local'):
                local_include_directories.append(path)
            continue
        include_directories.append(path)
    # Fail on any /usr/local include directories
    if local_include_directories:
        error_message = 'Found include directories under /usr/local: '
        error_message += ', '.join(local_include_directories)
        raise RuntimeError(error_message)

    defines = []
    ignored_defines = [
        # CMake always creates this to help with Win32 dllimport/export macros
        target.name + '_EXPORTS'
    ]
    for define in target.defines:
        if define in ignored_defines:
            continue
        defines.append(define)

    compile_flags = []
    ignored_compile_flags = [
        '-fPIC'  # applies to shared libraries only
    ]
    for flag in target.compile_flags:
        flag = flag.strip()
        if not flag or flag in ignored_compile_flags:
            continue
        compile_flags.append(flag)

    return CcProperties(
        include_directories = include_directories,
        compile_flags = compile_flags,
        defines = defines,
        link_libraries = link_libraries,
        link_flags = link_flags)


def collect_ament_cmake_package_properties(name, metadata):
    # NOTE(hidmic): each package properties are analyzed in isolation
    # to preclude potential interactions if multiple packages were
    # brought into the same CMake run. The latter could be done for
    # speed
    with TemporaryDirectory(dir=os.getcwd()) as project_path:
        project_name = 'empty_using_' + name
        cmakelists_template_path = path_to_resource(
            'templates/ament_cmake_CMakeLists.txt.in')
        cmakelists_path = os.path.join(project_path, 'CMakeLists.txt')
        cmake_tools.configure_file(cmakelists_template_path, cmakelists_path, {
            '@NAME@': project_name, '@PACKAGE@': name
        })

        cmake_prefix_path = os.path.realpath(metadata['prefix'])
        if 'AMENT_PREFIX_PATH' in os.environ:
            ament_prefix_path = os.environ['AMENT_PREFIX_PATH']
            cmake_prefix_path += ';' + ament_prefix_path.replace(':', ';')

        project_path = Path(project_path)
        build_path = project_path.joinpath('build')
        build_path.mkdir()
        try:
            codemodel = cmake_tools.get_cmake_codemodel(project_path, build_path)
        except RuntimeError as e:
            raise RuntimeError(f"Error occured while generating build for {name}")

        # Should only be one SHARED_LIBRARY target
        target = None
        for target in codemodel.targets:
            if 'SHARED_LIBRARY' == target.target_type:
                break

        assert target is not None

        additional_libraries = []
        if 'rosidl_interface_packages' in metadata.get('groups', []):
            # Pick up extra shared libraries in interface packages for
            # proper sandboxing
            glob_pattern = os.path.join(
                metadata['prefix'], 'lib', f'lib{name}__rosidl*.so')
            additional_libraries.extend(glob.glob(glob_pattern))
        properties = collect_ament_cmake_shared_library_codemodel(
            target, additional_libraries
        )
        return properties


def collect_ament_cmake_package_direct_properties(
    name, metadata, dependencies, cache
):
    if 'ament_cmake' not in cache:
        cache['ament_cmake'] = {}
    ament_cmake_cache = cache['ament_cmake']

    if name not in ament_cmake_cache:
        ament_cmake_cache[name] = \
            collect_ament_cmake_package_properties(name, metadata)

    properties = ament_cmake_cache[name]
    for dependency_name, dependency_metadata in dependencies.items():
        if dependency_metadata.get('build_type') != 'ament_cmake':
            continue
        if dependency_name not in ament_cmake_cache:
            ament_cmake_cache[dependency_name] = \
                collect_ament_cmake_package_properties(
                    dependency_name, dependency_metadata)
        dependency_properties = ament_cmake_cache[dependency_name]

        # Remove duplicates maintaining order.
        properties.compile_flags = tuple([
            flags for flags in properties.compile_flags
            if flags not in dependency_properties.compile_flags
        ])
        properties.defines = tuple([
            flags for flags in properties.defines
            if flags not in dependency_properties.defines
        ])
        properties.link_flags = tuple([
            flags for flags in properties.link_flags
            if flags not in dependency_properties.link_flags
        ])
        properties.link_libraries = tuple([
            library for library in properties.link_libraries
            if library not in dependency_properties.link_libraries
        ])
        deduplicated_include_directories = []
        for directory in properties.include_directories:
            if directory in dependency_properties.include_directories:
                # We may be dealing with a merge install.
                # Try leverage REP-122.
                if not os.path.exists(os.path.join(directory, name)):
                    continue
            deduplicated_include_directories.append(directory)
        properties.include_directories = tuple(deduplicated_include_directories)
        # Do not deduplicate link directories in case we're dealing with
        # merge installs.

    return properties


def precache_ament_cmake_properties(packages, jobs=None):
    ament_cmake_packages = {
        name: metadata
        for name, metadata in packages.items()
        if metadata.get('build_type') == 'ament_cmake'
    }
    with Pool(jobs) as pool:
        return dict(zip(
            ament_cmake_packages.keys(), pool.starmap(
                collect_ament_cmake_package_properties,
                ament_cmake_packages.items()
            )
        ))
