# -*- python -*-
# vi: set ft=python :

load(
    "//tools/skylark:dload.bzl",
    "dload_aware_target",
)

BASE_PORT = 16384

def to_bitstrings(integer, *bitsizes):
    bitstrings = []
    for bitsize in reversed(bitsizes):
        bitstrings.append(integer & ((1 << bitsize) - 1))
        integer = integer >> bitsize
    return reversed(bitstrings)

def rmw_fastrtps_netconf(name):
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

    profile = name + "_profile"
    native.genrule(
        name = profile,
        outs = [profile + ".xml"],
        cmd = "./$(location {}) {} > $@".format(tool, " ".join(args)),
        tools = [tool],
        output_to_bindir = True,
    )
    profile_path = "{}/{}/{}.xml".format(
        native.repository_name(), native.package_name(), profile
    )
    dload_aware_target(
        name = name,
        base = ":" + profile,
        runenv = {
            "FASTRTPS_DEFAULT_PROFILES_FILE": ["path-replace", profile_path],
        }
    )
