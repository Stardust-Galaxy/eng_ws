// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "msg_interfaces/msg/detail/angle__struct.h"
#include "msg_interfaces/msg/detail/angle__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool msg_interfaces__msg__angle__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("msg_interfaces.msg._angle.Angle", full_classname_dest, 31) == 0);
  }
  msg_interfaces__msg__Angle * ros_message = _ros_message;
  {  // found
    PyObject * field = PyObject_GetAttrString(_pymsg, "found");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->found = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // quaternion1
    PyObject * field = PyObject_GetAttrString(_pymsg, "quaternion1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quaternion1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quaternion2
    PyObject * field = PyObject_GetAttrString(_pymsg, "quaternion2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quaternion2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quaternion3
    PyObject * field = PyObject_GetAttrString(_pymsg, "quaternion3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quaternion3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // quaternion4
    PyObject * field = PyObject_GetAttrString(_pymsg, "quaternion4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->quaternion4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * msg_interfaces__msg__angle__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Angle */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("msg_interfaces.msg._angle");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Angle");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  msg_interfaces__msg__Angle * ros_message = (msg_interfaces__msg__Angle *)raw_ros_message;
  {  // found
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->found);
    {
      int rc = PyObject_SetAttrString(_pymessage, "found", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quaternion1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quaternion1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quaternion1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quaternion2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quaternion2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quaternion2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quaternion3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quaternion3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quaternion3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quaternion4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->quaternion4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quaternion4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
