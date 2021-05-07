import os

def configure(name, mapping):
    def sandbox(path, external=False):
        path = os.path.normpath(path)
        for outer_path, inner_path in mapping.items():
            if path.startswith(outer_path):
                path = os.path.normpath(
                    path.replace(outer_path, inner_path)
                )
                if external:
                    path = os.path.join(name, path)
                return path
        return path
    sandbox.kwargs = dict(name=name, mapping=mapping)

    for outer_path, inner_path in mapping.items():
        if inner_path == '.':
            continue
        os.symlink(outer_path, inner_path, target_is_directory=True)

    return sandbox
