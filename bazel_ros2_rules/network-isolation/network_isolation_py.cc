#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "network-isolation/network_isolation.h"

static PyObject *
create_linux_namespaces(PyObject *, PyObject *)
{
    if (!network_isolation::create_linux_namespaces()) {
        Py_RETURN_FALSE;
    }
    Py_RETURN_TRUE;
}

static PyMethodDef methods[] = {

    {"create_linux_namespaces", &create_linux_namespaces, METH_NOARGS,
     "Isolate the current process using linux namespaces."},
    {NULL, NULL, 0, NULL}   /* sentinel */
};

static PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "network_isolation_py",
    .m_doc = "Tools to isolate network traffic on linux.",
    .m_size = -1,
    &methods,
};

PyMODINIT_FUNC
PyInit_network_isolation_py(void)
{
    m = PyModule_Create(&module);
    if (m == NULL)
        return NULL;

    return m;
}
