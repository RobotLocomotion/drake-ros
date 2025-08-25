# Development Container

This directory contains the resources to build and run a containerized development environment for `drake-ros`.
This provides a consistent and reproducible environment with all necessary dependencies pre-installed.

## Support

This development environment is supported on:
  - Ubuntu Noble (24.04), both `amd64` and `arm64`
  - Apple M1 Macs

## Usage

There are several ways to use the development container, depending on your workflow.

### VSCode Dev Containers (Recommended)

This is the easiest and most integrated way to use the development environment.

1.  Make sure you have the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) installed in Visual Studio Code.
2.  Open the root of the `drake-ros` repository in VSCode.
3.  VSCode will detect the `.devcontainer` configuration and ask if you want to "Reopen in Container". Click on it.

VSCode will automatically build the Docker image, start the container, and connect to it, providing a seamless development experience.

### Plain Docker

You can build and run the container manually using standard Docker commands.

1.  **Build the image:** From the repository root, run:
    ```bash
    docker build -t RobotLocomotion/drake-ros -f .devcontainer/Dockerfile .
    ```

2.  **Run the container:**
    ```bash
    docker run --rm -v "$(pwd)":/drake-ros -w /drake-ros -it RobotLocomotion/drake-ros
    ```
    This will start the container, mount your local repository code into the container's workspace, and give you an interactive shell.

### Rocker (for GUI support)

If you need to run GUI applications from within the container (e.g., RViz), the `rocker` utility is a good choice as it simplifies X forwarding
and `nvidia` runtime setup. It also matches user and group IDs between container and host.

1.  First, [install rocker](https://github.com/osrf/rocker).
2.  Build the Docker image as shown in the previous section.
3.  Run the container using `rocker`:
    ```bash
    rocker --x11 --nvidia --user --volume "$(pwd)":/drake-ros RobotLocomotion/drake-ros
    ```

### Docker Compose (for GUI support without Nvidia GPUs)

A `docker-compose.yml` file is provided in the `tools` directory for convenience.

1.  Navigate to the root of the repository.
2.  Start the container in the background:
    ```bash
    docker-compose -f tools/docker-compose.yml up -d
    ```
3.  To open a shell inside the running container, use `docker exec`:
    ```bash
    docker exec -it drake_ros_container /bin/bash
    ```
    *(Note: You can find the container name using `docker ps`)*
4.  Go to `http://localhost:8080/vnc.html` to see any visualizations that you may run e.g. `rviz2`.
