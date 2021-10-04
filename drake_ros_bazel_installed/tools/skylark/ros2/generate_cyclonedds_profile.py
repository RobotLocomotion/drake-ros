import argparse
import sys

import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--discovery-multicast-address')
    parser.add_argument('--discovery-multicast-port-offset')
    parser.add_argument('--discovery-unicast-port-offset')
    parser.add_argument('--user-multicast-port-offset')
    parser.add_argument('--user-unicast-port-offset')
    parser.add_argument('--domain-gain')
    args = parser.parse_args()

    builder = ET.TreeBuilder()
    xmlns = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'
    schema_location = (
        'https://cdds.io/config https://raw.githubusercontent.com'
        '/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd')
    builder.start('CycloneDDS', {
        'xmlns:xsi': xmlns, 'xsi:schemaLocation': schema_location})
    builder.start('Domain', {'id': 'any'})
    builder.start('Discovery')
    if args.discovery_multicast_address:
        builder.start('SPDPMulticastAddress')
        builder.data(args.discovery_multicast_address)
        builder.end('SPDPMulticastAddress')
    builder.start('Ports')
    if args.domain_gain:
        builder.start('DomainGain')
        builder.data(args.domain_gain)
        builder.end('DomainGain')
    if args.discovery_multicast_port_offset:
        builder.start('MulticastMetaOffset')
        builder.data(args.discovery_multicast_port_offset)
        builder.end('MulticastMetaOffset')
    if args.user_multicast_port_offset:
        builder.start('MulticastDataOffset')
        builder.data(args.user_multicast_port_offset)
        builder.end('MulticastDataOffset')
    if args.discovery_unicast_port_offset:
        builder.start('UnicastMetaOffset')
        builder.data(args.discovery_unicast_port_offset)
        builder.end('UnicastMetaOffset')
    if args.user_unicast_port_offset:
        builder.start('UnicastDataOffset')
        builder.data(args.user_unicast_port_offset)
        builder.end('UnicastDataOffset')
    builder.end('Ports')
    builder.end('Discovery')
    builder.end('Domain')
    builder.end('CycloneDDS')
    tree = builder.close()
    inline_xml = ET.tostring(tree, encoding='utf-8')
    dom = minidom.parseString(inline_xml)
    pretty_xml = dom.toprettyxml(indent=' ' * 4, encoding='utf-8')
    sys.stdout.buffer.write(pretty_xml)


if __name__ == '__main__':
    main()
