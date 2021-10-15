# -*- python -*-

def incorporate_rmw_implementation(kwargs, env_changes, rmw_implementation):
    target = "@REPOSITORY_ROOT@:%s_cc" % rmw_implementation
    kwargs["data"] = kwargs.get("data", []) + [target]
    env_changes = dict(env_changes)
    env_changes.update({
         "RMW_IMPLEMENTATION": ["replace", rmw_implementation]
    })
    return kwargs, env_changes
