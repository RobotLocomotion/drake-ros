# drake-ros Docker README

This is a README for using `drake-ros` with docker. It supports the following platforms - 
  - Ubuntu 22.04 both `amd64` and `arm64`
  - Apple Silicon Macs: Via `BUILD_DRAKE_FROM_SOURCE`
The docker container would allow you to both visualize in `rviz2` (on M1 Macs as well via [noVNC](https://novnc.com/info.html)!) and develop.

## **Prerequisites:**
- Docker must be installed on your system. If you don't have Docker installed, follow the installation instructions at [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/).
- For NVIDIA GPU support, you need to have NVIDIA Container Toolkit installed. Follow the instructions at [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to install it.

  - Rocker: `rocker` installation is required for NVIDIA GPU support. A simple way to install it is - `pip3 install rocker` Find out more about it [here](https://github.com/osrf/rocker).
  - `nvidia-docker2`: Installation Instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html#id2) 
  

## Instructions:
1. Clone the repo and go to the directory containing the Dockerfile.
2. Build the Docker image by running either -
    ```
    docker build --build-arg ARCH=arm64v8/ --build-arg BUILD_DRAKE_FROM_SOURCE=true -t drake-ros:arm64 .
    (On arm64 systems, support is only available via building the source code.)
    
    docker build --build-arg ARCH=amd64/ --build-arg BUILD_DRAKE_FROM_SOURCE=false -t drake-ros:x86_64 .
    (Recommended for x86 systems since it's significantly faster unless you want to build the latest drake)
    ```

**NOTE**: When building `drake` from source make sure you are alotting at least 8 gigs of RAM and limiting the number of cores (rule of thumb is 1 job per 8GB RAM) accordingly to the docker container as well. Otherwise, your builds will fail. 

## Machines with Nvidia GPUs - 

1. Start a Docker container with the `drake-ros` image using `rocker`. The following command mounts the current working directory (the root of the `drake-ros` repository) inside the container at `/drake_ros_ws/src/`, enables NVIDIA GPU support, and sets up X11 forwarding:

    ```$ rocker --nvidia --x11 --volume "$(pwd):/drake_ros_ws/src/" -- drake-ros```

  If needed, replace `drake-ros` with the name of the Docker image you built earlier.
Alternatively, you can also run an equivalent `docker run` command.  

## Machines without NVIDIA GPUs (Integrated CPU graphics, Apple Silicon etc.) -

1. Use the provided docker compose via - `DRAKE_ROS_CONTAINER_NAME=my_custom_container_name docker compose up`
2. In a new terminal, run the following to get a bash session running in the
`drake-ros` container - `docker exec -it drake_ros_integration /bin/bash`
3. Go to `http://localhost:8080/vnc.html` to see any visualizations that you may run. Example - `rviz2`

Note - Since your drake-ros directory is mounted to the overlay ROS2 workspace in the container, any changes you make internally will be reflected on your host system. This is done to make development easier. You can read more about it [here](https://docs.docker.com/storage/volumes/). 
