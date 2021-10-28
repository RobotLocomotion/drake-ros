import hashlib
import os
import pathlib
import tempfile

import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

import rclpy

PARTICIPANT_ID_GAIN = 2
MAX_PARTICIPANTS_PER_PROCESS = 10

# 2 multicast ports + 2 unicast ports per participant
DISCOVERY_PORTS_INTERVAL = MAX_PARTICIPANTS_PER_PROCESS * PARTICIPANT_ID_GAIN + 2

def isolated_rmw_fastrtps_cpp_env(unique_identifier, scratch_directory=None):
    """
    Generates an environment that forces rmw_fastrtps_cpp network traffic isolation.

    This function achieves network traffic isolation by modifying multicast discovery
    IPv4 address and well-known ports offsets, which FastRTPS (now FastDDS) allows
    through the FASTRTPS_DEFAULT_PROFILES_FILE environment variable. For further reference,
    see https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/listening_locators.html.

    :param unique_identifier: unique arbitrary string to be used as a basis for isolation.
    :param scratch_directory: optional directory for generated files.
        If not provided, a temporary directory will be created.
    :returns: a dictionary of environment variables.
    """
    profile_name = unique_identifier
    transport_name = unique_identifier + '/transport'

    digest = hashlib.sha256(unique_identifier.encode('utf8')).digest()

    builder = ET.TreeBuilder()
    xmlns = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'
    builder.start('profiles', {'xmlns': xmlns})
    builder.start('transport_descriptors')
    builder.start('transport_descriptor')
    builder.start('transport_id')
    builder.data(transport_name)
    builder.end('transport_id')
    builder.start('type')
    builder.data('UDPv4')
    builder.end('type')
    builder.end('transport_descriptor')
    builder.end('transport_descriptors')
    builder.start('participant', {
        'profile_name': profile_name,
        'is_default_profile': 'true'
    })
    builder.start('rtps')
    builder.start('builtin')
    builder.start('metatrafficMulticastLocatorList')
    builder.start('locator')
    builder.start('udpv4')
    builder.start('address')
    multicast_discovery_ip_address = '.'.join(
        map(str, [239] + [int(c) for c in digest[:3]]))
    builder.data(multicast_discovery_ip_address)
    builder.end('address')
    builder.end('udpv4')
    builder.end('locator')
    builder.end('metatrafficMulticastLocatorList')
    builder.end('builtin')
    builder.start('port')
    builder.start('domainIDGain')
    builder.data('0')  # ignore domain IDs
    builder.end('domainIDGain')
    builder.start('participantIDGain')
    builder.data(str(PARTICIPANT_ID_GAIN))
    builder.end('participantIDGain')
    discovery_ports_offset = (
        int(digest[3]) * DISCOVERY_PORTS_INTERVAL)
    builder.start('offsetd0')
    builder.data(str(discovery_ports_offset))
    builder.end('offsetd0')
    builder.start('offsetd1')
    builder.data(str(discovery_ports_offset + 2))
    builder.end('offsetd1')
    builder.start('offsetd2')
    builder.data(str(discovery_ports_offset + 1))
    builder.end('offsetd2')
    builder.start('offsetd3')
    builder.data(str(discovery_ports_offset + 3))
    builder.end('offsetd3')
    builder.end('port')
    builder.start('userTransports')
    builder.start('transport_id')
    builder.data(transport_name)
    builder.end('transport_id')
    builder.end('userTransports')
    builder.start('useBuiltinTransports')
    builder.data('false')
    builder.end('useBuiltinTransports')
    builder.end('rtps')
    builder.end('participant')
    builder.end('profiles')
    tree = builder.close()
    inline_xml = ET.tostring(tree, encoding='utf-8')
    dom = minidom.parseString(inline_xml)
    pretty_xml = dom.toprettyxml(indent=' ' * 4, encoding='utf-8')
    if scratch_directory is None:
        scratch_directory = tempfile.mkdtemp()
    profiles_path = pathlib.Path(scratch_directory) / 'fastrtps_profiles.xml'
    with profiles_path.open('wb') as f:
        f.write(pretty_xml)
    return {'FASTRTPS_DEFAULT_PROFILES_FILE': str(profiles_path), 'ROS_DOMAIN_ID': '0'}

def isolated_rmw_cyclonedds_cpp_env(unique_identifier, scratch_directory=None):
    """
    Generates an environment that forces rmw_cyclonedds_cpp network traffic isolation.

    This function achieves network traffic isolation by modifying multicast discovery
    IPv4 address and domain ID, which CycloneDDS allows through the CYCLONEDDS_URI
    environment variable. For further reference, see
    https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst.

    :param unique_identifier: unique arbitrary string to be used as a basis for isolation.
    :param scratch_directory: optional directory for generated files.
        If not provided, a temporary directory will be created.
    :returns: a dictionary of environment variables.
    """
    digest = hashlib.sha256(unique_identifier.encode('utf8')).digest()

    builder = ET.TreeBuilder()
    xmlns = 'http://www.w3.org/2001/XMLSchema-instance'
    schema_location = (
        'https://cdds.io/config https://raw.githubusercontent.com'
        '/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd')
    builder.start('CycloneDDS', {
        'xmlns:xsi': xmlns, 'xsi:schemaLocation': schema_location})
    builder.start('Domain', {'id': 'any'})
    builder.start('Discovery')
    builder.start('SPDPMulticastAddress')
    multicast_discovery_ip_address = '.'.join(
        map(str, [239] + [int(c) for c in digest[:3]]))
    builder.data(multicast_discovery_ip_address)
    builder.end('SPDPMulticastAddress')
    builder.start('ParticipantIndex')
    builder.data('none')  # disable unicast discovery
    builder.end('ParticipantIndex')
    builder.start('Ports')
    builder.start('DomainGain')
    builder.data('225')  # ensure all 256 domain IDs are valid
    builder.end('DomainGain')
    builder.end('Ports')
    builder.end('Discovery')
    builder.end('Domain')
    builder.end('CycloneDDS')
    tree = builder.close()
    inline_xml = ET.tostring(tree, encoding='utf-8')
    dom = minidom.parseString(inline_xml)
    pretty_xml = dom.toprettyxml(indent=' ' * 4, encoding='utf-8')
    if scratch_directory is None:
        scratch_directory = tempfile.mkdtemp()
    configuration_path = pathlib.Path(scratch_directory) / 'cyclonedds_configuration.xml'
    with configuration_path.open('wb') as f:
        f.write(pretty_xml)
    return {
        'CYCLONEDDS_URI': f'file://{configuration_path.resolve()}',
        'ROS_DOMAIN_ID': str(int(digest[3]))}

def isolated_rmw_env(unique_identifier, rmw_implementation=None, scratch_directory=None):
    """
    Generates an environment that forces rmw implementation network traffic isolation.

    :param unique_identifier: unique arbitrary string to be used as
        a basis for isolation.
    :param rmw_implementation: optional target rmw implementation.
        If not provided, the currently applicable implementation
        will be used, as returned by rclpy.get_rmw_implementation_identifier().
    :param scratch_directory: optional directory for generated files, if any.
        If not provided, a temporary directory will be created.
    :returns: a dictionary of environment variables.
    :raises ValueError: if the target rmw implementation is unknown.
    """
    if rmw_implementation is None:
        rmw_implementation = rclpy.get_rmw_implementation_identifier()
    target_function_name = f'isolated_{rmw_implementation}_env'
    if target_function_name not in globals():
        raise ValueError(f'cannot isolate unknown {rmw_implementation} implementation')
    return globals()[target_function_name](
        unique_identifier, scratch_directory=scratch_directory)

def isolate_rmw_by_path(path):
    """
    Isolates rmw implementation network traffic.

    This function relies on `isolated_rmw_env` to populate
    the calling process environment and achieve network isolation.

    :param path: unique path to use as a basis for isolation.
    :raises RuntimeError: if called after rmw initialization.
    """
    if rclpy.ok():
        raise RuntimeError('middleware already initialized, too late for isolation')
    os.environ.update(isolated_rmw_env(str(path), scratch_directory=path))
