# -*- python -*-
# vi: set ft=python :

BASE_PORT = 16384

def to_bitstrings(integer, *bitsizes):
    bitstrings = []
    for bitsize in reversed(bitsizes):
        bitstrings.append(integer & ((1 << bitsize) - 1))
        integer = integer >> bitsize
    return reversed(bitstrings)

def generate_isolated_fastrtps_profile(name):
    tool = "//tools/skylark/ros2:generate_fastrtps_profile"
    label_name = "{}//{}:{}".format(
        native.repository_name(), native.package_name(), name
    )
    bitstrings = to_bitstrings(hash(label_name), 8, 8, 8, 12, 12, 12)
    args = [
        name,
        "--metatraffic-multicast-locator {}:{}".format(
            ".".join(["239"] + [str(octet) for octet in bitstrings[:3]]),
            BASE_PORT + bitstrings[3]
        ),
        "--metatraffic-unicast-locator {}".format(BASE_PORT + bitstrings[4]),
        "--unicast-locator {}".format(BASE_PORT + bitstrings[5]),
        "--use-as-default",
    ]

    native.genrule(
        name = "gen_" + name,
        outs = [name],
        cmd = "./$(location {}) {} > $@".format(tool, " ".join(args)),
        tools = [tool],
        output_to_bindir = True,
    )

def generate_isolated_cyclonedds_profile(name):
    tool = "//tools/skylark/ros2:generate_cyclonedds_profile"
    label_name = "{}//{}:{}".format(
        native.repository_name(), native.package_name(), name
    )
    bitstrings = to_bitstrings(hash(label_name), 8, 8, 8, 10, 10, 10, 10)
    args = [
        "--discovery-multicast-address {}".format(
            ".".join(["239"] + [str(octet) for octet in bitstrings[:3]])),
        "--discovery-multicast-port-offset {}".format(bitstrings[3]),
        "--user-multicast-port-offset {}".format(bitstrings[4]),
        "--discovery-unicast-port-offset {}".format(bitstrings[5]),
        "--user-unicast-port-offset {}".format(bitstrings[6]),
        "--domain-gain {}".format(0),  # ignore domain IDs
    ]

    native.genrule(
        name = "gen_" + name,
        outs = [name],
        cmd = "./$(location {}) {} > $@".format(tool, " ".join(args)),
        tools = [tool],
        output_to_bindir = True,
    )
