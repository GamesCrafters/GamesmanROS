// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from gamesmanros_interfaces:msg/PieceDetection.idl
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
#include "gamesmanros_interfaces/msg/detail/piece_detection__struct.h"
#include "gamesmanros_interfaces/msg/detail/piece_detection__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool gamesmanros_interfaces__msg__piece_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
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
    assert(strncmp("gamesmanros_interfaces.msg._piece_detection.PieceDetection", full_classname_dest, 58) == 0);
  }
  gamesmanros_interfaces__msg__PieceDetection * ros_message = _ros_message;
  {  // tag_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "tag_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tag_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // cell_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "cell_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cell_index = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // board_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "board_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->board_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // board_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "board_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->board_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // offset_mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "offset_mm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->offset_mm = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * gamesmanros_interfaces__msg__piece_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PieceDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("gamesmanros_interfaces.msg._piece_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PieceDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  gamesmanros_interfaces__msg__PieceDetection * ros_message = (gamesmanros_interfaces__msg__PieceDetection *)raw_ros_message;
  {  // tag_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->tag_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tag_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cell_index
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->cell_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cell_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // board_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->board_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "board_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // board_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->board_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "board_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // offset_mm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->offset_mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offset_mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
