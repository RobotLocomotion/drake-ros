# -*- python -*-

def incorporate_rmw_implementation(kwargs, env_changes, rmw_implementation):
    target = "@REPOSITORY_ROOT@:%s_cc" % rmw_implementation
    kwargs["data"] = kwargs.get("data", []) + [target]
    env_changes = dict(env_changes)
    env_changes.update({
         "RMW_IMPLEMENTATION": ["replace", rmw_implementation]
    })
    return kwargs, env_changes

def incorporate_fastrtps_profile(kwargs, env_changes, profile_name):        
    kwargs["data"] = kwargs.get("data", []) + [profile_name]
    profile_path = "{}/{}".format(native.package_name(), profile_name)
    if native.repository_name() != "@":
        repository_name = native.repository_name().lstrip("@")
        profile_path = "{}/{}".format(repository_name, profile_path)
    env_changes = dict(env_changes)
    env_changes.update({
        "FASTRTPS_DEFAULT_PROFILES_FILE": ["path-replace", profile_path]
    })
    return kwargs, env_changes

def incorporate_cyclonedds_profile(kwargs, env_changes, profile_name):
    kwargs["data"] = kwargs.get("data", []) + [profile_name]
    profile_path = "{}/{}".format(native.package_name(), profile_name)
    if native.repository_name() != "@":
        repository_name = native.repository_name().lstrip("@")
        profile_path = "{}/{}".format(repository_name, profile_path)
    profile_uri = "file://$PWD/" + profile_path
    env_changes = dict(env_changes)
    env_changes.update({"CYCLONEDDS_URI": ["replace", profile_uri]})
    return kwargs, env_changes
