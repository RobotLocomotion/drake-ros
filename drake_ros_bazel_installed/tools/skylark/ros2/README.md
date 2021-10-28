## Infrastructure to use ROS 2 from a Bazel workspace

This package encapsulates all the machinery to pull a ROS 2 workspace install space or a subset thereof as a Bazel repository.
Both system-installed binary distributions and source builds can pulled in this way, whether symlink- or merged-installed.

A single repository rule, `ros2_local_repository()`, is the sole entrypoint. This rule heavily relies on two Python packages:

- the `cmake_tools` Python package, which provides an idiomatic API to collect a CMake project's exported configuration.
- the `ros2bzl` Python package, which provides tools to crawl a ROS 2 workspace install space, collects CMake packages' 
  exported configuration, collect Python packages' egg metadata, symlink relevant directories and files, and generates
  a root BUILD.bazel file that recreates the dependency graph in the workspace. This package constitutes the backbone of
  the `generate_repository_files.py` Python binary which `ros2_local_repository()` invokes.

### Repository layout

A ROS 2 local repository has the following layout:

```
  .
  ├── BUILD.bazel
  ├── rmw_isolation
  │   └── BUILD.bazel
  ├── distro.bzl
  ├── common.bzl
  ├── ros_cc.bzl
  ├── ros_py.bzl
  └── rosidl.bzl
```

Note that all files and subdirectories that are mere implementation details have been excluded from this layout. 

#### Targets

For every package in the underlying ROS 2 workspace install space, the following targets may be found at the root `BUILD.bazel` file:

- A `<package_name>_share` filegroup for every package that installs artifacts under the share directory.
- A `<package_name>_cc` C/C++ library for every CMake package that installs C++ binary artifacts (in addition to compiler/linker flags, includes, etc.).
- A `<package_name>_py` Python library for every package that installs a Python egg.
- A `<package_name>_defs` filegroup for every package that installs ROS 2 interfaces (.msg, .srv, .action, and .idl files). 
- A `<package_name>_c` C/C++ library for every CMake package that installs C binary artifacts. Typically an alias of the `<package>_cc` C/C++ library if C and C++ binary artifacts cannot be told apart.
- A `<package_name>_transitively_py` Python library for every package that does not install a Python egg yet it depends and it is a dependency of packages that do. This helps maintain the dependency graph.
- A `<package_name>_<executable_name>` for every executable installed at the package-level (i.e. under `lib/<package>`, where `ros2 run` can find them).
- A `<executable_name>` for every executable installed under the `bin` directory (and thus in the `$PATH`).

#### Rules

To build C++ binaries and tests that depend on ROS 2, `ros_cc_binary` and `ros_cc_test` rules are available in the `ros_cc.bzl` file. These rules, equivalent to the native `cc_binary` and `cc_test` rules, ensure these binaries run in a environment that is tightly coupled with the underlying ROS 2 workspace install space.

To build Python binaries and tests that depend on ROS 2, `ros_py_binary` and `ros_pytest` rules are available in the `ros_py.bzl` file. These rules, equivalent to the native `py_binary` and `py_test` rules, ensure these binaries run in a environment that is tightly coupled with the underlying ROS 2 workspace install space.

To generate and build ROS 2 interfaces, a `rosidl_interfaces_group` rule is available in the `rosidl.bzl` file. This rule generates C++ and Python code for the given ROS 2 interface definition files and builds them, off of what is available in the ROS 2 workspace install space: code generators, interface definition translators, runtime dependencies, etc. Several targets are created, following strict naming conventions (e.g. C++ and Python interface libraries carry `_cc` and `_py` suffixes, respectively), though finer-grained control over what is generated and built can be achieved through via other rules available in the same file. By default, these naming conventions allow downstream `rosidl_interfaces_group` rules to depend on upstream `rosidl_interface_group` rules.

#### Tools

The `rmw_isolation` subpackage provides C++ and Python `isolate_rmw_by_path` APIs to enforce RMW network isolation. To that end, a unique path must be provided (such as Bazel's `$TEST_TMPDIR`).

#### Metadata

The `distro.bzl` file bears relevant ROS 2 workspace metadata for rules, tools, and downstream packages to use.
