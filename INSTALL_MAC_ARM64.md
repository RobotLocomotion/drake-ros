# drake-ros: Docker Installation on Mac ARM64 (Apple Silicon)

This guide documents the verified steps to build and run the drake-ros Docker
development environment on Apple Silicon (M-series) Macs.

**Tested with:** Docker Desktop 4.64.0, Ubuntu Noble 24.04, ROS 2 Jazzy,
Drake v1.43.0, Bazel 9.0.1 on Apple M-series (aarch64).

---

## Prerequisites

### 1. Install Docker Desktop

```bash
brew install --cask docker
```

If you do not have admin/sudo access, copy Docker.app manually:

```bash
# Find the cached .app from Homebrew
ls /opt/homebrew/Caskroom/docker-desktop/*/Docker.app

# Copy to your user Applications folder
cp -R /opt/homebrew/Caskroom/docker-desktop/*/Docker.app ~/Applications/Docker.app
```

### 2. Start Docker Desktop

```bash
open ~/Applications/Docker.app    # or /Applications/Docker.app
```

Wait for Docker to fully start (whale icon in the menu bar becomes steady).
Verify with:

```bash
docker info
```

You should see `OSType: linux` and `Architecture: aarch64`.

---

## ARM64 Fixes Applied to the Dockerfile

The stock `.devcontainer/Dockerfile` is written for x86_64. The following
changes were made for ARM64 compatibility. **These changes are already applied
in the repository's Dockerfile.**

### Fix 1: fixuid dynamic architecture detection

The original Dockerfile hardcoded `linux-amd64` for the fixuid download.
Changed to detect architecture dynamically:

```dockerfile
RUN USER=ubuntu && \
    GROUP=ubuntu && \
    ARCH=$(dpkg --print-architecture) && \
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-${ARCH}.tar.gz | tar -C /usr/local/bin -xzf - && \
    ...
```

### Fix 2: libquadmath0 virtual package

`libquadmath0` is an x86-only GCC package that Drake's prerequisites require.
On ARM, we create a virtual (dummy) package with `equivs`:

```dockerfile
RUN if [ "$(dpkg --print-architecture)" != "amd64" ]; then \
      sudo apt-get update && \
      sudo apt-get install -y equivs && \
      printf "Package: libquadmath0\nVersion: 99.0\nArchitecture: all\nDescription: virtual package for ARM64 compat\n" > /tmp/libquadmath0.ctl && \
      cd /tmp && equivs-build libquadmath0.ctl && \
      sudo dpkg -i /tmp/libquadmath0_99.0_all.deb && \
      sudo rm -rf /tmp/libquadmath0* /var/lib/apt/lists/*; \
    fi
```

### Fix 3: sysctl wrapper for Docker build

Docker builds on ARM fail because
`/proc/sys/kernel/apparmor_restrict_unprivileged_unconfined` does not exist.
Certain package install scripts invoke `sysctl` and crash. We temporarily wrap
`sysctl` to suppress these errors:

```dockerfile
RUN sudo dpkg-divert --local --rename --add /usr/sbin/sysctl && \
    printf '#!/bin/sh\n/usr/sbin/sysctl.distrib "$@" 2>/dev/null || true\n' | sudo tee /usr/sbin/sysctl > /dev/null && \
    sudo chmod +x /usr/sbin/sysctl
```

After installing prerequisites, restore the original:

```dockerfile
RUN sudo rm -f /usr/sbin/sysctl && \
    sudo dpkg-divert --local --rename --remove /usr/sbin/sysctl 2>/dev/null; true
```

### Fix 4: Use `--with-bazel` only (skip `--with-colcon`)

On ARM, `--with-colcon` triggers a full from-source CMake build of Drake which
takes many hours and can exhaust the Docker VM's memory (default 8 GB). Using
`--with-bazel` only means Drake is fetched by Bazel at build time instead:

```dockerfile
RUN yes | /tmp/drake-ros/setup/install_prereqs \
     -C /tmp/throwaway --with-bazel && \
   sudo rm -rf /tmp/throwaway /var/lib/apt/lists/*
```

---

## ARM64 Fixes Applied to `setup/install_prereqs`

These fix two outdated paths in the `install_drake_from_sources()` function.
They are only exercised if you use `--with-colcon` (not needed for the default
Bazel workflow).

### Fix 5: Drake prerequisites script path

```diff
- ./drake/setup/ubuntu/install_prereqs --without-bazel
+ ./drake/setup/install_prereqs --without-bazel
```

### Fix 6: CMake source directory

```diff
- cmake -DCMAKE_INSTALL_PREFIX=/opt/drake ..
+ cmake -DCMAKE_INSTALL_PREFIX=/opt/drake ../drake
```

---

## Build the Docker Image

From the repository root:

```bash
cd /path/to/drake-ros

docker build \
  --platform linux/arm64 \
  -t RobotLocomotion/drake-ros \
  -f .devcontainer/Dockerfile .
```

This takes approximately 10–15 minutes depending on network speed.
The resulting image is ~6.8 GB.

---

## Run a Container

> **How to tell where you are:**
> - **Mac host terminal** — your prompt looks like `yourusername@MacBook-Pro` or similar. `docker` commands work here.
> - **Inside the container** — your prompt looks like `ubuntu@<container-id>:/drake-ros$`. `docker` is NOT available here and will show `bash: docker: command not found`.

### 🖥️ From your Mac host — start an interactive container

```bash
# Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros

docker run --rm \
  --platform linux/arm64 \
  -v "$(pwd)":/drake-ros \
  -w /drake-ros \
  --privileged \
  -it RobotLocomotion/drake-ros
```

Your prompt will change to `ubuntu@<id>:/drake-ros$` — you are now inside the container.

### 📦 Inside the container — verify the environment

```bash
# Run inside the container (prompt: ubuntu@<id>:...)
uname -m
# Expected: aarch64

source /opt/ros/jazzy/setup.bash
ros2 --help
# Expected: ROS 2 CLI help output

bazel --version
# Expected: bazel 9.0.1
```

### With GUI (noVNC) via docker-compose

> Run all of these on your **Mac host terminal**.

```bash
# Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros/tools
docker-compose -f docker-compose.yml up -d
docker exec -it drake_ros_container /bin/bash
```

Then open <http://localhost:8080/vnc.html> in your browser for a virtual desktop.

---

## Building and Running Examples

Each top-level package (`drake_ros_examples`, `drake_ros`, `ros2_example_bazel_installed`)
has its own Bazel workspace. You must `cd` into the package directory before
running Bazel commands.

> **Important — ARM64 memory limit:** The Docker Desktop VM defaults to 8 GB
> RAM. Use `--jobs=4` to limit Bazel parallelism and avoid OOM kills during
> compilation.

### rs_flip_flop example

#### Option A: Interactive (recommended)

**Step 1 — On your Mac host,** start a container:

```bash
# 🖥️ Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros

docker run --rm \
  --platform linux/arm64 \
  -v "$(pwd)":/drake-ros \
  -w /drake-ros/drake_ros_examples \
  --privileged \
  -it RobotLocomotion/drake-ros
```

**Step 2 — Inside the container** (prompt changes to `ubuntu@<id>:/drake-ros/drake_ros_examples$`):

```bash
# 📦 Run inside the container
source /opt/ros/jazzy/setup.bash

# Build (first build fetches Drake — takes a few minutes)
bazel build --jobs=4 //examples/rs_flip_flop:rs_flip_flop

# Run
bazel run --jobs=4 //examples/rs_flip_flop:rs_flip_flop
```

If you are already in a running container but in the wrong directory:

```bash
# 📦 Run inside the container
cd /drake-ros/drake_ros_examples
source /opt/ros/jazzy/setup.bash
bazel build --jobs=4 //examples/rs_flip_flop:rs_flip_flop
bazel run --jobs=4 //examples/rs_flip_flop:rs_flip_flop
```

#### Option B: One-liner from Mac host

```bash
# 🖥️ Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros

docker run --rm \
  --platform linux/arm64 \
  -v "$(pwd)":/drake-ros \
  -w /drake-ros/drake_ros_examples \
  --privileged \
  -it RobotLocomotion/drake-ros \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           bazel run --jobs=4 //examples/rs_flip_flop:rs_flip_flop"
```

### multirobot example (with RViz)

The multirobot example visualises a 5×5 array of Kuka iiwa arms flopping
under gravity. It requires a display for RViz, so you must use the **noVNC**
docker-compose stack instead of a plain `docker run`.

#### Step 1 — 🖥️ Mac host: start the noVNC compose stack

```bash
# Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros

# Start the drake_ros container + noVNC display server
docker-compose -f tools/docker-compose.yml up -d
```

Open <http://localhost:8080/vnc.html> in your browser — you should see a
black virtual desktop. This is the display RViz will render into.

#### Step 2 — 🖥️ Mac host: open two shells into the container

Open **two separate Mac terminals** and run this in each:

```bash
# Run on your Mac terminal (NOT inside a container)
docker exec -it drake_ros_container bash
```

You now have two shells inside `drake_ros_container`.

#### Step 3 — 📦 Shell 1 (inside container): start RViz

```bash
# Run inside the container
cd /drake-ros/drake_ros_examples
source /opt/ros/jazzy/setup.bash

# Launch RViz with the pre-configured multirobot layout
bazel run --jobs=4 @ros2//:rviz2 -- -d $(pwd)/examples/multirobot/multirobot.rviz
```

RViz will open in the noVNC browser tab. Wait until it is fully loaded before
continuing.

#### Step 4 — 📦 Shell 2 (inside container): start the simulation

```bash
# Run inside the container
cd /drake-ros/drake_ros_examples
source /opt/ros/jazzy/setup.bash

# C++ version
bazel run --jobs=4 //examples/multirobot:multirobot

# OR Python version
bazel run --jobs=4 //examples/multirobot:multirobot_py
```

You should see the 5×5 array of robot arms appear in RViz and begin flopping
under gravity.

> **Note:** If you restart the simulation without restarting RViz, click the
> **Reset** button in RViz to clear stale TF data.

#### Step 5 — 🖥️ Mac host: stop the stack when done

```bash
# Run on your Mac terminal (NOT inside a container)
cd /path/to/drake-ros
docker-compose -f tools/docker-compose.yml down
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `fixuid` 404 during build | Ensure the `ARCH=$(dpkg --print-architecture)` fix is applied |
| `libquadmath0 has no installation candidate` | Ensure the `equivs` virtual package block is present |
| `sysctl: cannot stat /proc/sys/kernel/apparmor_restrict_unprivileged_unconfined` | Ensure the sysctl wrapper is in place |
| Build hangs for hours | Make sure `--with-colcon` is NOT passed; use `--with-bazel` only |
| Docker Desktop won't start | Grant necessary permissions in System Settings > Privacy & Security |
| `dpkg-divert: error: rename involves overwriting` | The sysctl restore step must `rm -f` the wrapper before `dpkg-divert --remove` |
| `gcc: fatal error: Killed signal terminated program cc1plus` | OOM — reduce parallelism with `--jobs=4 --local_ram_resources=6144` |
| `ERROR: no such package '//drake_ros_examples/...'` | You must `cd` into the package directory first (e.g. `cd drake_ros_examples`) |

---

## Summary of Modified Files

| File | Changes |
|------|---------|
| `.devcontainer/Dockerfile` | Fixes 1–4: ARM arch detection, libquadmath0 virtual pkg, sysctl wrapper, bazel-only build |
| `setup/install_prereqs` | Fixes 5–6: Updated Drake source paths (only needed for `--with-colcon`) |
