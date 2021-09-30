import argparse
import sys

import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET


def build_udpv4_locator(builder, locator):
    builder.start('locator')
    builder.start('udpv4')
    ip, _, port = locator.rpartition(':')
    if ip:
        builder.start('address')
        builder.data(ip)
        builder.end('address')
    builder.start('port')
    builder.data(port)
    builder.end('port')
    builder.end('udpv4')
    builder.end('locator')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('profile_name')
    parser.add_argument(
        '--use-as-default', action='store_true', default=False
    )
    parser.add_argument(
        '--metatraffic-multicast-locator',
        dest='metatraffic_multicast_locators',
        action='append', default=[]
    )
    parser.add_argument(
        '--metatraffic-unicast-locator',
        dest='metatraffic_unicast_locators',
        action='append', default=[]
    )
    parser.add_argument(
        '--unicast-locator',
        dest='unicast_locators',
        action='append', default=[]
    )
    args = parser.parse_args()

    transport_name = args.profile_name + '_transport'

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
        'profile_name': args.profile_name,
        'is_default_profile': str(args.use_as_default).lower()
    })
    builder.start('rtps')
    if args.metatraffic_multicast_locators or \
       args.metatraffic_unicast_locators:
        builder.start('builtin')
        if args.metatraffic_multicast_locators:
            builder.start('metatrafficMulticastLocatorList')
            for locator in args.metatraffic_multicast_locators:
                build_udpv4_locator(builder, locator)
            builder.end('metatrafficMulticastLocatorList')
        if args.metatraffic_unicast_locators:
            builder.start('metatrafficUnicastLocatorList')
            for locator in args.metatraffic_unicast_locators:
                build_udpv4_locator(builder, locator)
            builder.end('metatrafficUnicastLocatorList')
        builder.end('builtin')
    if args.unicast_locators:
        builder.start('defaultUnicastLocatorList')
        for locator in args.unicast_locators:
            build_udpv4_locator(builder, locator)
        builder.end('defaultUnicastLocatorList')
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
    sys.stdout.buffer.write(pretty_xml)


if __name__ == '__main__':
    main()
