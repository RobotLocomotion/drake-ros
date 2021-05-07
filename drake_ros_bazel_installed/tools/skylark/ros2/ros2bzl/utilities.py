import functools

def to_starlark_string_dict(d):
    # Replace single-quotes with double-quotes
    return {k: repr(v).replace("'", '"') for k, v in d.items()}

def interpolate(template, config):
    return template.format(**to_starlark_string_dict(config))

def compose(f, g):
    @functools.wraps(f)
    def wrapped(head, *args, **kwargs):
        return f(g(head, *args, **kwargs), *args, **kwargs)
    return wrapped
