# -*- python -*-

def rosidl_generate_interfaces_command(
    repository, package, interfaces, output, prefix=None, dependencies=[]
):
    cmd = ["ROS2BZL_PREFIX_PATH={repository_dir}", "{repository_dir}/env.sh"]
    cmd.append("{repository_dir}/generate_rosidl_file.py")
    cmd.extend(["-o ", output])
    if prefix:
        cmd.extend(["-t", prefix])
    cmd.append(repository)
    cmd.append(package)
    cmd.extend(interfaces)
    cmd.extend(["-d " + dep for dep in dependencies])
    return " ".join(cmd)
