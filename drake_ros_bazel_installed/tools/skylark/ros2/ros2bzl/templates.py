import os

from ros2bzl.resources import load_resource
from ros2bzl.scrapping.system import find_library_path

def load_paths_for_libraries(libraries, sandbox):
    if not libraries:
        return []
    libraries_directories = sorted(set(
        sandbox(os.path.dirname(lib), external=True) for lib in libraries
    ))
    load_paths = [libraries_directories[0]]
    for directory in libraries_directories[1:]:
        if directory.startswith(load_paths[-1]):
            continue
        load_paths.append(directory)
    return load_paths


def label_name(name, metadata):
    return metadata.get('bazel_name', name)


def label(name, metadata):
    workspace = metadata.get('bazel_workspace', '')
    if workspace:
        workspace = '@' + workspace
    package = metadata.get('bazel_package', '')
    if workspace or package:
        package = '//' + package
    return workspace + package + ':' + label_name(name, metadata)


def labels_with(suffix):
    return (
        (lambda *args, **kwargs: label_name(*args, **kwargs) + suffix),
        (lambda *args, **kwargs: label(*args, **kwargs) + suffix)
    )


share_name, share_label = labels_with(suffix='_share')
c_name, _ = labels_with(suffix='_c')
cc_name, cc_label = labels_with(suffix='_cc')
py_name, py_label = labels_with(suffix='_py')
meta_py_name, meta_py_label = labels_with(suffix='_transitively_py')


def configure_package_share_filegroup(name, metadata, sandbox):
    return load_resource('package_share_filegroup.bzl.tpl'), {
        'name': share_name(name, metadata),
        'share_directories': [
            sandbox(metadata['share_directory']),
            sandbox(metadata['ament_index_directory']),
        ]
    }

def configure_package_interfaces_filegroup(name, metadata, sandbox):
    return load_resource('package_interfaces_filegroup.bzl.tpl'), {
        'name': name, 'share_directory': sandbox(metadata['share_directory'])
    }


def configure_package_cc_library(name, metadata, properties, dependencies, extras, sandbox):
    target_name = cc_name(name, metadata)
    libraries = [sandbox(library) for library in properties['link_libraries']]
    include_directories = [sandbox(include) for include in properties['include_directories']]
    local_includes = [include for include in include_directories if not os.path.isabs(include)]
    # Assume package abides to REP-122 FHS layout
    headers = [os.path.join(include, name) for include in local_includes]
    # Push remaining nonlocal includes through compiler options
    copts = ['-isystem ' + include for include in include_directories if os.path.isabs(include)]
    copts.extend(properties['compile_flags'])
    defines = properties['defines']

    linkopts = properties['link_flags']
    for link_directory in properties['link_directories']:
        link_directory = sandbox(link_directory)
        if not link_directory:
            continue
        linkopts += [
            '-L' + link_directory,
            '-Wl,-rpath ' + link_directory
        ]
    deps = [
        cc_label(dependency_name, dependency_metadata)
        for dependency_name, dependency_metadata in dependencies.items()
        if 'cc' in dependency_metadata['langs']
    ]

    runenv = {'AMENT_PREFIX_PATH': [
        'path-prepend', sandbox(metadata['prefix'], external=True)
    ]}
    if 'rmw_implementation_packages' in metadata['groups']:
        runenv['RMW_IMPLEMENTATION'] = ['replace', name]

    data = []
    data.append(share_label(name, metadata))
    # Add in plugins, if any
    if 'plugin_libraries' in metadata:
        data.extend(
            sandbox(find_library_path(library))
            for library in metadata['plugin_libraries']
        )
    # Prepare runfiles and load paths to support dynamic loading
    load_paths = load_paths_for_libraries(
        properties['link_libraries'], sandbox
    )
    if load_paths:
        runenv['${LOAD_PATH}'] = ['path-prepend', *load_paths]
    data.extend(library for library in libraries if library not in data)
    if extras and 'data' in extras and target_name in extras['data']:
        data.extend(
            label_or_path
            for label_or_path in extras['data'][target_name]
            if label_or_path not in data
        )

    return load_resource('package_cc_library.bzl.tpl'), {
        'name': target_name,
        'srcs': libraries,
        'headers': headers,
        'includes': local_includes,
        'copts': copts,
        'defines': defines,
        'linkopts': linkopts,
        'runenv': runenv,
        'data': data,
        'deps': deps,
    }


def configure_package_meta_py_library(name, metadata, dependencies):
    deps = []
    for dependency_name, dependency_metadata in dependencies.items():
        if 'py' in dependency_metadata['langs']:
            deps.append(py_label(dependency_name, dependency_metadata))
        elif 'py (transitively)' in dependency_metadata['langs']:
            deps.append(meta_py_label(dependency_name, dependency_metadata))
    return load_resource('package_meta_py_library.bzl.tpl'), {
        'name': meta_py_name(name, metadata), 'deps': deps
    }


def configure_package_py_library(name, metadata, properties, dependencies, extras, sandbox):
    target_name = py_name(name, metadata)
    eggs = [sandbox(egg_path) for egg_path, _ in properties['python_packages']]
    tops = [sandbox(top_level) for _, top_level in properties['python_packages']]
    imports = [os.path.dirname(egg) for egg in eggs]

    deps = []
    for dependency_name, dependency_metadata in dependencies.items():
        if 'py' in dependency_metadata['langs']:
            deps.append(py_label(dependency_name, dependency_metadata))
        elif 'py (transitively)' in dependency_metadata['langs']:
            deps.append(meta_py_label(dependency_name, dependency_metadata))

    config = {
        'name': target_name,
        'tops': tops,
        'eggs': eggs,
        'imports': imports,
        'deps': deps
    }

    data = [share_label(name, metadata)]
    if 'cc' in metadata['langs']:
        data.append(cc_label(name, metadata))

    runenv = {'AMENT_PREFIX_PATH': [
        'path-prepend', sandbox(metadata['prefix'], external=True)
    ]}

    if 'rmw_implementation_packages' in metadata['groups']:
        runenv['RMW_IMPLEMENTATION'] = ['replace', name]

    if 'cc_extensions' in properties:
        cc_deps = [
            cc_label(dependency_name, dependency_metadata)
            for dependency_name, dependency_metadata in dependencies.items()
            if 'cc' in dependency_metadata['langs']
        ]
        cc_extensions = [sandbox(ext) for ext in properties['cc_extensions']]
        # Prepare runfiles and load paths to support dynamic loading
        load_paths = load_paths_for_libraries(properties['cc_extensions'], sandbox)
        if load_paths:
            runenv['${LOAD_PATH}'] = ['path-prepend', *load_paths]
        data.extend(cc_extensions)
        data.extend(cc_deps)
        # Add in plugins, if any
        if 'plugin_libraries' in metadata:
            data.extend(
                sandbox(find_library_path(library))
                for library in metadata['plugin_libraries']
            )
        template = load_resource('package_py_library_with_cc_extensions.bzl.tpl')
    else:
        template = load_resource('package_py_library.bzl.tpl')

    if extras and 'data' in extras:
        data.extend(extras['data'].get(target_name, []))
    config.update({'data': data, 'runenv': runenv})

    return template, config


def configure_package_alias(name, target):
    return load_resource('package_alias.bzl.tpl'), {
        'name': name, 'actual': ':' + target
    }


def configure_package_c_library_alias(name, metadata):
    return load_resource('package_alias.bzl.tpl'), {
        'name': c_name(name, metadata),
        'actual': cc_label(name, metadata)
    }


def configure_executable_imports(
    executables, dependencies, sandbox, extras=None, prefix=None
):
    deps = []
    common_data = []
    for dependency_name, dependency_metadata in dependencies.items():
        # TODO(hidmic): use appropriate target based on executable file type
        if 'cc' in dependency_metadata['langs']:
            common_data.append(cc_label(dependency_name, dependency_metadata))
        if 'py' in dependency_metadata['langs']:
            deps.append(py_label(dependency_name, dependency_metadata))
        elif 'py (transitively)' in dependency_metadata['langs']:
            common_data.append(meta_py_label(dependency_name, dependency_metadata))

    for executable in executables:
        target_name = os.path.basename(executable)
        if prefix:
            target_name = prefix + '_' + target_name
        data = common_data
        if extras and 'data' in extras and target_name in extras['data']:
            data = data + extras['data'][target_name]
        yield load_resource('executable_import.bzl.tpl'), {
            'name': target_name,
            'executable': sandbox(executable),
            'data': data, 'deps': deps,
        }


def configure_package_executable_imports(
    name, metadata, dependencies, sandbox, extras=None
):
    dependencies = dict(dependencies)
    dependencies[name] = metadata
    yield from configure_executable_imports(
        metadata['executables'], dependencies, sandbox,
        extras=extras, prefix=label_name(name, metadata)
    )
