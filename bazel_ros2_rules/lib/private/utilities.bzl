# -*- python -*-

def find_local_ros2_distribution(ctx):
    prefix = ctx.getenv("ROS_DISTRO_PREFIX")
    if not prefix:
        distro_name = ctx.getenv("ROS_DISTRO")
        if not distro_name:
            result = ctx.execute(["ls", "/opt/ros"])
            if result.return_code != 0 or result.stdout == "":
                fail("Found no ROS 2 distributions, use $ROS_DISTRO_PREFIX to locate one")
            distro_names = result.stdout.splitlines()
            if len(distro_names) > 1:
                fail("Found many ROS 2 distributions, use $ROS_DISTRO to choose one", distro_names)
            distro_name = distro_names[0]
        prefix = "/opt/ros/" + distro_name
    return prefix