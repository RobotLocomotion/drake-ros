import os
import xml.etree.ElementTree as ET


def parse_package_xml(path_to_package_xml):
    tree = ET.parse(path_to_package_xml)

    depends = set([
        tag.text for tag in tree.findall('./depend')
    ])
    exec_depends = set([
        tag.text for tag in tree.findall('./exec_depend')
    ])
    buildtool_depends = set([
        tag.text for tag in tree.findall('./buildtool_depend')
    ])
    build_export_depends = set([
        tag.text for tag in tree.findall('./build_export_depend')
    ])
    group_depends = set([
        tag.text for tag in tree.findall('./group_depend')
    ])
    member_of_groups = set([
        tag.text for tag in tree.findall('./member_of_group')
    ])
    build_type = tree.find('./export/build_type').text

    return dict(
        build_dependencies=build_export_depends | depends | buildtool_depends,
        run_dependencies=exec_depends | depends,
        group_dependencies=group_depends,
        groups=member_of_groups,
        build_type=build_type
    )


def parse_plugins_description_xml(path_to_plugins_description_xml):
    plugins_description_xml = ET.parse(path_to_plugins_description_xml)
    root = plugins_description_xml.getroot()
    assert root.tag == 'library'
    return dict(plugin_libraries=[root.attrib['path']])


def find_executables(base_path):
    for dirpath, dirnames, filenames in os.walk(base_path):
        # ignore folder starting with .
        dirnames[:] = [d for d in dirnames if d[0] not in ['.']]
        dirnames.sort()
        # select executable files
        for filename in sorted(filenames):
            path = os.path.join(dirpath, filename)
            if os.access(path, os.X_OK):
                yield path


DEFAULT_LANGS_PER_BUILD_TYPE = {
    'cmake': {'cc'},
    'ament_cmake': {'cc'},
    'ament_python': {'py'},
}


def collect_package_langs(metadata):
    langs = set()
    build_type = metadata.get('build_type')
    if build_type in DEFAULT_LANGS_PER_BUILD_TYPE:
        langs.update(DEFAULT_LANGS_PER_BUILD_TYPE[build_type])
    return langs


def collect_cmake_package_metadata(name, prefix):
    metadata = dict(
        prefix=prefix,
        build_type='cmake',
    )
    metadata['langs'] = collect_package_langs(metadata)
    return metadata


def collect_ros_package_metadata(name, prefix):
    share_directory = os.path.join(prefix, 'share', name)
    ament_index_directory = os.path.join(prefix, 'share', 'ament_index')

    metadata = dict(
        prefix=prefix,
        share_directory=share_directory,
        ament_index_directory=ament_index_directory,
    )

    lib_directory = os.path.join(prefix, 'lib', name)
    metadata['executables'] = list(find_executables(lib_directory))

    path_to_package_xml = os.path.join(share_directory, 'package.xml')
    metadata.update(parse_package_xml(path_to_package_xml))

    metadata['langs'] = collect_package_langs(metadata)

    path_to_plugins_description_xml = os.path.join(
        share_directory, 'plugins_description.xml'
    )
    if os.path.exists(path_to_plugins_description_xml):
        metadata.update(parse_plugins_description_xml(
            path_to_plugins_description_xml
        ))

    return metadata
