#include <pybind11/pybind11.h>
#include "network_isolation.h"

namespace ros2 {
namespace ros2_py {

namespace py = pybind11;

PYBIND11_MODULE(network_isolation_py, m) {
    m.doc() = "Python wrapper for network isolation";

    m.def("create_linux_network_namespaces", &ros2::CreateLinuxNetworkNamespaces,
          "Creates isolated Linux network namespaces.");
}

}  // namespace ros2_py
} // namespace ros2
