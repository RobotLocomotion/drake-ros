from ros2bzl.scraping.ament_cmake import collect_ament_cmake_package_properties
from ros2bzl.scraping.python import collect_python_package_properties

# Alias
collect_ament_python_package_properties = collect_python_package_properties


def collect_ament_python_package_direct_properties(
    name, metadata, dependencies, cache
):
    if 'ament_python' not in cache:
        cache['ament_python'] = {}
    ament_python_cache = cache['ament_python']

    if name not in ament_python_cache:
        ament_python_cache[name] = \
            collect_ament_python_package_properties(name, metadata)

    properties = ament_python_cache[name]

    if properties.cc_libraries:
        ament_cmake_cache = cache['ament_cmake']
        for dependency_name, dependency_metadata in dependencies.items():
            dependency_libraries = []
            if 'cc' in dependency_metadata.get('langs', []):
                if dependency_metadata.get('build_type') == 'ament_cmake':
                    if dependency_name not in ament_cmake_cache:
                        ament_cmake_cache[dependency_name] = \
                            collect_ament_cmake_package_properties(
                                dependency_name, dependency_metadata)
                    dependency_properties = ament_cmake_cache[dependency_name]
                    dependency_libraries.extend(
                        dependency_properties.link_libraries
                    )
            if 'py' in dependency_metadata.get('langs', []):
                if dependency_name not in ament_python_cache:
                    ament_python_cache[dependency_name] = \
                        collect_ament_python_package_properties(
                            dependency_name, dependency_metadata)
                dependency_properties = ament_python_cache[dependency_name]
                if dependency_properties.cc_libraries:
                    dependency_libraries.extend(
                        dependency_properties.cc_libraries)
            # Remove duplicates maintaining order
            properties.cc_libraries = tuple([
                lib for lib in properties.cc_libraries
                if lib not in dependency_libraries
            ])
    return properties
