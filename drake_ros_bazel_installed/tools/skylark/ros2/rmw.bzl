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
    tool = "//tools/skylark/ros2:rmw_fastrtps_profile_gen"
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
