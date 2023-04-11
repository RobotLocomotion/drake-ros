# Drake ROS Examples

This is a collection of examples built around `drake_ros` libraries' C++ and Python APIs.

## Building with Colcon

This package is built and tested on Ubuntu Jammy 22.04 with ROS 2 Humble and Rolling,
using a recent Drake stable release.

It may work on other versions of ROS and Drake, but that is not explicitly tested.

To build it for Jammy with ROS 2 Humble using `colcon` and `ament` (ROS 2's build
tooling and CMake infrastructure)

1. [Install ROS Humble](https://docs.ros.org/en/humble/Installation.html) \
   Taken from the website (be sure to review these commands before executing!)

   ```sh
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   # Install the desktop user entry point.
   sudo apt install ros-humble-desktop
   # Install dev tools.
   sudo apt install ros-dev-tools

   # Update dependencies index.
   rosdep update
   ```

   For other entry points aside from `ros-humble-desktop`, please see the
   Humble section of REP 2001: \
   <https://www.ros.org/reps/rep-2001.html#humble-hawksbill-may-2022-ongoing>

1. Source your ROS installation

    ```sh
    source /opt/ros/humble/setup.bash
    ```

1. [Install the Drake](https://drake.mit.edu/installation.html), preferably a stable release, either locally
   or in a way that is global to your system. \
   You will need **both** the C++ and Python components of Drake, so installing via `pip` is not recommended.

   - For CMake, you should ensure Drake is on your `PATH`, or Drake's is on `CMAKE_PREFIX_PATH`.
   - For Python, you should ensure it is on your `PYTHONPATH`.

   - As an *example* installation method, here's how to install via `apt`: \
     https://drake.mit.edu/apt.html#stable-releases \
     (You may want to practice with a container first!)

      ```sh
      sudo apt-get update
      sudo apt-get install --no-install-recommends \
          ca-certificates gnupg lsb-release wget
      wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - \
          | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null
      echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
          | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null
      sudo apt-get update
      sudo apt-get install --no-install-recommends drake-dev
      ```

      In each terminal session, you should ensure the suggestioned environment variables
      are present (e.g. via `~/.bash_aliases` or your own `drake_setup.sh` script):

      ```sh
      export PATH="/opt/drake/bin${PATH:+:${PATH}}"
      export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
      ```

1. Build your workspace using Colcon.

    ```sh
    # Make a workspace
    # See also: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
    mkdir -p ./ws/src
    cd ./ws/src

    # Get this code
    git clone https://github.com/RobotLocomotion/drake-ros.git

    # Return to workspace
    cd ..

    # Install required dependencies.
    rosdep install --from-paths src -ryi

    # Build the packages.
    # Use --symlink-install so you can do things like edit Python source code
    # and see it reflected without rebuilding / reinstalling.
    colcon build --packages-up-to drake_ros_examples --symlink-install

    # Run the unittests - avoid surprises!
    colcon test --packages-up-to drake_ros_examples --event-handlers console_cohesion+
    # Note: If you encounter test errors, you may want to reinspect the results with
    # a bit more narrow focus.
    colcon test-result --verbose
    ```

**Note**: As you are developing, you may want to track your workspace using
[`vcstool`](https://github.com/dirk-thomas/vcstool) and/or use an existing `*.repos`
to initialize it. \
For an example set of repositories, see the (very comprehensive)
[`ros2.repos` per the ROS 2 Docs](https://docs.ros.org/en/humble/Installation/Maintaining-a-Source-Checkout.html)

## Running

Source your workspace.

```sh
source ./ws/install/setup.bash
```

Now you can run C++ and Python examples using nominal ROS 2 + ament + CMake
tooling per the list below.

## Examples

- [ROS 2 and Drake Systems Example - Flip-Flop](./examples/rs_flip_flop)
- [IIWA manipulator](./examples/iiwa_manipulator): an RViz visualization of a static IIWA arm.
- [Multi-Robot Simulation](./examples/multirobot): Simulates multiple robots
  using Drake and visualizes them with RViz using a single marker display topic.
