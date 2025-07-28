# Infrastructure to use ROS 2 from a Bazel Workspace

This package encapsulates all the machinery to pull a ROS 2 workspace install
space or a subset thereof as a Bazel repository. Both system-installed binary
distributions and source builds can pulled in this way, whether symlink- or
merged-installed.

A single repository rule, `ros2_local_repository()`, is the sole entrypoint.
This rule heavily relies on two Python packages:

- The `cmake_tools` Python package, which provides an idiomatic API to collect
  a CMake project's exported configuration (see Note 1).
- The `ros2bzl` Python package, which provides tools to crawl a ROS 2 workspace
  install space, collects CMake packages' exported configuration, collect Python
  packages' egg metadata, symlink relevant directories and files, and generates
  a root BUILD.bazel file that recreates the dependency graph in the workspace.
  This package constitutes the backbone of the `generate_repository_files.py`
  Python binary which `ros2_local_repository()` invokes.

**Note 1**
: [`rules_foreign_cc`](https://github.com/bazelbuild/rules_foreign_cc) tooling
  was initially considered but later discarded, as it serves a fundamentally
  different purpose. This infrastructure does **not** build CMake projects
  within Bazel (like `rules_foreign_cc` does), it exposes the artifacts and
  build configuration of pre-installed CMake projects for Bazel packages to
  depend on.

## Repository Layout

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

Note that all files and subdirectories that are mere implementation details
have been excluded from this layout.

### Targets

For each package in the underlying ROS 2 workspace install space, depending on
the artifacts it generates, the following targets may be found at the root
`BUILD.bazel` file:

- A `<package_name>_share` filegroup for files in the `<package_prefix>/share`
  directory but excluding those files that are build system specific.
- A `<package_name>_cc` C/C++ library for C++ libraries, typically found in
  the `<package_prefix>/lib` directory. In addition to headers, compiler flags,
  linker flags, etc., C/C++ library targets for upstream packages that are
  immediate dependencies are declared as dependencies themselves.
- A `<package_name>_py` Python library for Python eggs in the
  `<package_prefix>/lib/python*/site-packages` directory. All Python library
  targets for upstream packages that are immediate dependencies are declared
  as dependencies themselves.
- A `<package_name>_defs` filegroup for interface definition files (.msg, .srv,
  .action, and .idl files) in the `<package_prefix>/share` directory.
- A `<package_name>_c` C/C++ library for C libraries. Typically an alias of the
  `<package>_cc` target if C and C++ libraries cannot be told apart.
- A `<package_name>_transitive_py` Python library if the package does not
  install any Python libraries but it depends on (and it is a dependency of)
  packages that do. This helps maintain the dependency graph (as Python library
  targets can only depend on other Python library targets).
- A `<package_name>_<executable_name>` Python binary for each executable
  installed at the package-level (i.e. under `lib/<package_name>`, where
  `ros2 run` can find them).
- An `<executable_name>` Python binary wrapper for each executable of *any*
  kind (Python, shell script, compiled binary, etc.) installed under the
  `<package_prefix>/bin` directory (and thus accessible via `$PATH` when
  sourcing the workspace install space). These executables are exposed as
  Python binaries for simplicty.

### Limitations

`bazel_ros2_rules` heavily relies on a number of conventions and best
practices that _most_ ROS 2 packages follow to compensate for the limited
amount of metadata that ROS 2 install spaces carry.

[REP-0122](https://ros.org/reps/rep-0122.html) is but one example of this.

This means that for ROS 2 packages that break these conventions and best
practices, `bazel_ros2_rules` output will degrade (if not break down).
An example of this are ROS 2 packages that install CMake and/or Python
packages with names other than their own. These will be found but lacking
dependency information any generated rules (`cc_library`, `py_library`, etc.)
will be completely disconnected from the rest. Users that need them will be
forced to depend on them explicitly.

### Rules

To build C++ binaries and tests that depend on ROS 2, `ros_cc_binary` and
`ros_cc_test` rules are available in the `ros_cc.bzl` file. These rules,
equivalent to the native `cc_binary` and `cc_test` rules, ensure these binaries
run in an environment that is tightly coupled with the underlying ROS 2
workspace install space.

To build Python binaries and tests that depend on ROS 2, `ros_py_binary` and
`ros_pytest` rules are available in the `ros_py.bzl` file. These rules,
equivalent to the native `py_binary` and `py_test` rules, ensure these binaries
run in an environment that is tightly coupled with the underlying ROS 2
workspace install space.

To generate a Python or XML launch target, `ros_launch` is available in the
`ros_py.bzl` file. Note that there are some nuances; pelase see the Launch
Files section below.

To generate and build ROS 2 interfaces, a `rosidl_interfaces_group` rule is
available in the `rosidl.bzl` file. This rule generates C++ and Python code
for the given ROS 2 interface definition files and builds them using what is
available in the ROS 2 workspace install space: code generators, interface
definition translators, runtime dependencies, etc. Several targets are created,
following strict naming conventions (e.g. C++ and Python interface libraries
carry `_cc` and `_py` suffixes, respectively), though finer-grained control over
what is generated and built can be achieved through other rules available in
the same file. By default, these naming conventions allow downstream
`rosidl_interfaces_group` rules to depend on upstream `rosidl_interface_group`
rules.

#### Launch Files

An example of the `ros_launch` macro can be found under `ros2_example_apps`.
Please note the following limitations:

- Exposing a Bazel package as a ROS package has not yet been done. Once that is
  done, `launch_ros.actions.Node` / `<node/>` can be used.
- For Python launch files, it is best to use `Rlocation` (as shown in the example)
  so that the launch file can be run via `bazel run`, `bazel test`, and
  `./bazel-bin` (directly).
- For XML launch files, we need to (somehow) expose either ROS packages or
  `Rlocation`. This needs to be done in a way that can be discovered by
  [`Parser.load_launch_extensions`](https://github.com/ros2/launch/blob/698e979382877242621a0d633750fe96ff0c2bca/launch/launch/frontend/parser.py#L72-L87),
  which may require care to do so correctly in Bazel.

### Tools

The `rmw_isolation` subpackage provides C++ and Python `isolate_rmw_by_path`
APIs to enforce RMW network isolation. To that end, a unique path must be
provided (such as Bazel's `$TEST_TMPDIR`).

**DISCLAIMER**
: Isolation relies on `rmw`-specific configuration. Support is available for
  Tier 1 `rmw` implementations only. Collision rates are below 1% but not null.
  Use with care.

### Metadata

The `distro.bzl` file bears relevant ROS 2 workspace metadata for rules, tools,
and downstream packages to use.

## Alternatives

### `mvukov/rules_ros2`

[`mvukov/rules_ros2`](https://github.com/mvukov/rules_ros2) provides an elegant
way of manually mapping a [`vcstool`](https://github.com/dirk-thomas/vcstool),
e.g. [`ros2.repos`](https://github.com/ros2/ros2/blob/rolling/ros2.repos), onto
a custom but simple manifest to then bootstrap into Bazel repository rules (and
the corresponding `BUILD` files) to build from source.

Additionally, affordances are provided to leverage Python `pip` provisiong (via
[`rules_python`](https://github.com/bazelbuild/rules_python/)) and examples of
Docker container integration (via
[`rules_docker`](https://github.com/bazelbuild/rules_docker)).

The benefit from this approach is being able to fully control the source build,
configuration flags, etc., for ensuring a build is consistent and easily
reconfigurable.

The (present) possible con to this approach is scalability. You must presently
provide your own set of external `{repo}.BUILD.bazel` rules that reflect the
ament operations expressed in CMake. Until the `ament` / ROS 2 build ecosystem
provides scaffolding or affordances for Bazel tooling, and until packages
provide their own (tested) Bazel tooling, this may be a tall (but certainly
tractable) hill to climb.

Additionally, some entry points need to be reflected into the codebase (e.g.
`ros2_topic.py`), which provides for nice flexibility, but at the cost of
duplication / redundancy w.r.t. upstream.

### `ApexAI/rules_ros`

[`ApexAI/rules_ros`](https://github.com/ApexAI/rules_ros) is another great
method of ingesting ROS 2 into a Bazel-ified workflow. At present, it has a
`vcstool`-like manifest of
repositories that will be used to fetch packages and build them from source.

The pros and cons to this approach are similar to `mvukov/rules_ros2`.

### This Repository (`RobotLocomotion/drake-ros/bazel_ros2_rules`)

`RobotLocomotion/drake-ros/bazel_ros2_rules` leverages a prebuilt workspace, be
it from a local path, a tarball, or some other Bazel `repository_rule`
provenance (e.g.
[`RobotLocomotion/bazel-external-data`](https://github.com/RobotLocomotion/bazel-external-data).

The pro is that only scraping needs to be
done of CMake metadata, and thus can be more scalable, especially for complex
package ecosystems like RViz2 related things.

The con is that this scraping can be slow, is currently in the analysis phase,
and does not admit source-level configurability.

#### Features

From the above two examples (at present), the following features are in
`RobotLocomotion/bazel_ros2_rules` that may be missing from the others:

- Host-based isolation (see `rmw_isolation` tooling here)
- Ament index analysis (e.g. using `ros2 interface list` in Bazel and seeing
  your custom types)
- Examples leverage RViz from within a Bazel workspace
- Examples include Python bindings (via `pybind11`) against other C++ libraries
  that have their own bindings - e.g., [Drake](https://drake.mit.edu/).
  - There is *some* affordance for leveraging `pybind11` against ROS 2 RMW
    libraries. However, because the Python bindings do not leverage C++
    directly, but instead leverage the C-level interfaces, we get into an
    awkward ground of mutually exclusive memory / resource management
    paradigms.

The other repos, however, have the following that `bazel_ros2_rules` is
missing:

- Launching from containers (Docker, Apptainer)

Some common features might be:

- Bazel affordances for `ros2 launch`
