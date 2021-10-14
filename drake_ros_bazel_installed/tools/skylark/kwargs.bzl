# -*- python -*-

_COMMON_KWARGS = [
    "compatible_with",
    "deprecation",
    "exec_compatible_with",
    "exec_properties",
    "features",
    "restricted_to",
    "tags",
    "target_compatible_with",
    "testonly",
    "toolchains",
    "visibility",
]

def keep_common(kwargs):
    """Fetch keyword arguments common to all rules from `kwargs`."""
    return {key: value for key, value in kwargs.items() if key in _COMMON_KWARGS}
