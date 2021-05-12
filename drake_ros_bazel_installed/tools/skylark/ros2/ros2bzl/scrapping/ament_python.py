import glob

from importlib.metadata import distribution
from importlib.metadata import PackageNotFoundError

from ros2bzl.scrapping.system import find_library_dependencies
from ros2bzl.scrapping.system import is_system_library


def find_python_package(name):
    dist = distribution(name)
    top_level = dist.read_text('top_level.txt')
    top_level = top_level.rstrip('\n')
    return str(dist.locate_file(top_level))


def collect_ament_python_package_properties(name, metadata):
    python_package_path = find_python_package(name)
    properties = {'python_packages': [python_package_path]}
    cc_extensions = glob.glob(
        '{}/**/*.so'.format(python_package_path), recursive=True
    )
    if cc_extensions:
        cc_extensions_deps = set()
        for ext in cc_extensions:
            cc_extensions_deps.update([
                library
                for library in find_library_dependencies(ext)
                if not is_system_library(library)
            ])
        properties['cc_extensions'] = cc_extensions
        properties['cc_extensions'] += list(cc_extensions_deps)
    return properties


def collect_ament_python_package_direct_properties(name, metadata, dependencies, cache):
    if 'ament_python' not in cache:
        cache['ament_python'] = {}
    ament_python_cache = cache['ament_python']

    if name not in ament_python_cache:
        ament_python_cache[name] = \
            collect_ament_python_package_properties(name, metadata)

    properties = dict(ament_python_cache[name])

    if 'cc_extensions' in properties:
        ament_cmake_cache = cache['ament_cmake']
        for dependency_name, dependency_metadata in dependencies.items():
            dependency_libraries = []
            if 'cc' in dependency_metadata['langs']:
                if dependency_metadata['build_type'] == 'ament_cmake':
                    if dependency_name not in ament_cmake_cache:
                        ament_cmake_cache[dependency_name] = \
                            collect_ament_cmake_package_properties(
                                dependency_name, dependency_metadata
                            )
                    dependency_properties = ament_cmake_cache[dependency_name]
                    dependency_libraries.extend(
                        dependency_properties['link_libraries']
                    )
            if 'py' in dependency_metadata['langs']:
                if dependency_name not in ament_python_cache:
                    ament_python_cache[dependency_name] = \
                        collect_ament_python_package_properties(
                            dependency_name, dependency_metadata
                        )
                dependency_properties = ament_python_cache[dependency_name]
                if 'cc_extensions' in dependency_properties:
                    dependency_libraries.extend(
                        dependency_properties['cc_extensions'])
            # Remove duplicates maintaining order
            properties['cc_extensions'] = [
                ext for ext in properties['cc_extensions']
                if ext not in dependency_libraries
            ]
    return properties
