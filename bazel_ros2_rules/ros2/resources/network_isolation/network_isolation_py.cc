#include <Python.h>
#include "network_isolation.h"

// Method wrapper definitions
static PyObject* create_linux_network_namespaces(PyObject *, PyObject *) {
    ros2::CreateLinuxNetworkNamespaces();
    Py_RETURN_NONE;
}

// Method definition table
static PyMethodDef network_isolation_py_methods[] = {{
    "create_linux_network_namespaces",
    &create_linux_network_namespaces, METH_NOARGS,
    "Creates isolated Linux network namespaces."},
    {NULL, NULL, 0, NULL} // Sentinel value ending the table
};

// Module definition structure
static struct PyModuleDef network_isolation_py_def = {
    PyModuleDef_HEAD_INIT,
    "network_isolation_py", // Module name
    "Python bindings for network isolation", // Module documentation
    -1, // Size of per-interpreter state or -1
    network_isolation_py_methods
};

// Module initialization function
PyMODINIT_FUNC PyInit_network_isolation_py(void) {
  return PyModule_Create(&network_isolation_py_def);
}
