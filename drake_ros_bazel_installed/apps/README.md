## Demo apps

This package exercises `rosidl` interface generation and usage in C++ and Python, with cross-package interface dependencies.

To check that the resulting interfaces are functional, it includes a demo applications in C++ and Python. Namely:

- `oracle_cc`/`oracle_py` applications that publish `apps_msgs/msg/Status` messages, serve a `common_msgs/srv/Query`
  service, and provide a `common_msgs/action/Do` action.
- `inquirer_cc`/`inquirer_py` applications that subscribe to `apps_msgs/msg/Status` messages, make `common_msgs/srv/Query`
  service requests, and invoke the `common_msgs/action/Do` action.
