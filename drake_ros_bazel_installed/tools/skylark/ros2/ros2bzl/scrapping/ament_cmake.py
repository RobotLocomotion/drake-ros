import glob
import os

from multiprocessing.dummy import Pool
from tempfile import TemporaryDirectory

import cmake_tools

from ros2bzl.resources import path_to_resource

from ros2bzl.scrapping.system import find_library_path
from ros2bzl.scrapping.system import find_library_dependencies
from ros2bzl.scrapping.system import is_system_include
from ros2bzl.scrapping.system import is_system_library


def collect_ament_cmake_shared_library_codemodel(codemodel):
    assert codemodel['type'] == 'SHARED_LIBRARY'

    link_flags = []
    if 'linkFlags' in codemodel:
        for flag in codemodel['linkFlags'].split(' '):
            flag = flag.strip()
            if not flag:
                continue
            link_flags.append(flag)

    link_libraries = []
    if 'linkLibraries' in codemodel:
        libraries = []
        for item in codemodel['linkLibraries'].split(' '):
            item = item.strip()
            if item.startswith('-') and not item.startswith('-l'):
                link_flags.append(item)
                continue
            libraries.append(item)
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
            link_libraries.extend([
                library for library in library_plus_dependencies
                if library not in link_libraries and not is_system_library(library)
            ])

    file_groups = codemodel['fileGroups']
    assert len(file_groups) == 1
    file_group = file_groups[0]

    include_directories = []
    if 'includePath' in file_group:
        include_directories.extend([
            entry['path'] for entry in file_group['includePath']
            if not is_system_include(entry['path'])
        ])

    defines = []
    if 'defines' in file_group:
        ignored_defines = [
            codemodel['name'] + '_EXPORTS'  # modern CMake specific
        ]
        for define in file_group['defines']:
            if define in ignored_defines:
                continue
            defines.append(define)

    compile_flags = []
    if 'compileFlags' in file_group:
        ignored_compile_flags = [
            '-fPIC'  # applies to shared libraries only
        ]
        for flag in file_group['compileFlags'].split(' '):
            flag = flag.strip()
            if not flag or flag in ignored_compile_flags:
                continue
            compile_flags.append(flag)

    return {
        'include_directories': include_directories,
        'compile_flags': compile_flags,
        'defines': defines,
        'link_directories': [],  # no directories in codemodel?
        'link_libraries': link_libraries,
        'link_flags': link_flags
    }


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

        try:
            with cmake_tools.server_mode(project_path) as cmake:
                cmake.configure(attributes={'cacheArguments': [
                    '-DCMAKE_PREFIX_PATH="{}"'.format(cmake_prefix_path)
                ]}, timeout=30, message_callback=print)
                cmake.compute(timeout=20, message_callback=print)
                codemodel = cmake.codemodel(timeout=10)
        except:
            import shutil
            shutil.rmtree('error_case', ignore_errors=True)
            shutil.copytree(project_path, 'error_case')
            raise

        configurations = codemodel['configurations']
        assert len(configurations) == 1
        configuration = configurations[0]

        projects = configuration['projects']
        assert len(projects) == 1
        project = projects[0]

        targets = {t['name']: t for t in project['targets']}
        assert project_name in targets
        target = targets[project_name]

        properties = collect_ament_cmake_shared_library_codemodel(target)
        if 'rosidl_interface_packages' in metadata.get('groups', []):
            # Pick up extra shared libraries in interface
            # packages for adequate sandboxing
            glob_pattern = os.path.join(
                metadata['prefix'], 'lib', f'lib{name}__rosidl*.so')
            for library in glob.glob(glob_pattern):
                if library not in properties['link_libraries']:
                    properties['link_libraries'].append(library)
        return properties


def collect_ament_cmake_package_direct_properties(name, metadata, dependencies, cache):
    if 'ament_cmake' not in cache:
        cache['ament_cmake'] = {}
    ament_cmake_cache = cache['ament_cmake']

    if name not in ament_cmake_cache:
        ament_cmake_cache[name] = \
            collect_ament_cmake_package_properties(name, metadata)

    properties = dict(ament_cmake_cache[name])
    for dependency_name, dependency_metadata in dependencies.items():
        if dependency_metadata.get('build_type') != 'ament_cmake':
            continue
        if dependency_name not in ament_cmake_cache:
            ament_cmake_cache[dependency_name] = \
                collect_ament_cmake_package_properties(
                    dependency_name, dependency_metadata)
        dependency_properties = ament_cmake_cache[dependency_name]

        # Remove duplicates maintaining order.
        properties['compile_flags'] = [
            flags for flags in properties['compile_flags']
            if flags not in dependency_properties['compile_flags']
        ]
        properties['defines'] = [
            flags for flags in properties['defines']
            if flags not in dependency_properties['defines']
        ]
        properties['link_flags'] = [
            flags for flags in properties['link_flags']
            if flags not in dependency_properties['link_flags']
        ]
        properties['link_libraries'] = [
            library for library in properties['link_libraries']
            if library not in dependency_properties['link_libraries']
        ]
        properties['include_directories'] = [
            directory for directory in properties['include_directories']
            if directory not in dependency_properties['include_directories']
            # leverage REP-122
            or os.path.exists(os.path.join(directory, name))
        ]
        # Do not deduplicate link directories in case we're dealing with merge installs.

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
