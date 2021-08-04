# -*- python -*-

DEFAULT_COMMON_KWARGS = {
    "compatible_with": [],
    "deprecation": None,
    "exec_compatible_with": [],
    "exec_properties": {},
    "features": [],
    "restricted_to": None,
    "tags": [],
    "target_compatible_with": [],
    "testonly": False,
    "toolchains": [],
    "visibility": []
}

def keep_common(kwargs):
    return {
        key: kwargs.get(key, value)
        for key, value in DEFAULT_COMMON_KWARGS.items()
    }
