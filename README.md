# drake_ros

The repository currently contains experimental code.

The intended function of this repository:
 - API to utilize ROS from a Drake program
 - Examples of how to use both Drake and ROS together


## Docker
### Building and Testing with Docker
$ docker build -t drake-ros-rolling -f tools/containers/drake_ros_core.Dockerfile  --build-arg ROS_DISTRO=rolling .

### Interactive Shell in Docker container
$ docker run -it --user root drake-ros  /bin/bash