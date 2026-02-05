// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
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
#include "quadrotor_msgs/msg/detail/px4ctrl_debug__struct.h"
#include "quadrotor_msgs/msg/detail/px4ctrl_debug__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool quadrotor_msgs__msg__px4ctrl_debug__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("quadrotor_msgs.msg._px4ctrl_debug.Px4ctrlDebug", full_classname_dest, 46) == 0);
  }
  quadrotor_msgs__msg__Px4ctrlDebug * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // des_v_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_v_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_v_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_v_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_v_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_v_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_v_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_v_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_v_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_a_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_a_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_a_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_a_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_a_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_a_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_a_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_a_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_a_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_a_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_a_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_a_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_a_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_a_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_a_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_a_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_a_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_a_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_q_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_q_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_q_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_q_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_q_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_q_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_q_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_q_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_q_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_q_w
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_q_w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_q_w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_thr
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_thr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_thr = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thr2acc
    PyObject * field = PyObject_GetAttrString(_pymsg, "thr2acc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thr2acc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // thr_scale_compensate
    PyObject * field = PyObject_GetAttrString(_pymsg, "thr_scale_compensate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->thr_scale_compensate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->voltage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_axisang_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_axisang_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_axisang_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_axisang_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_axisang_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_axisang_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_axisang_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_axisang_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_axisang_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_axisang_ang
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_axisang_ang");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_axisang_ang = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_rate_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_rate_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_rate_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_rate_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_rate_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_rate_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fb_rate_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "fb_rate_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fb_rate_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * quadrotor_msgs__msg__px4ctrl_debug__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Px4ctrlDebug */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("quadrotor_msgs.msg._px4ctrl_debug");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Px4ctrlDebug");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  quadrotor_msgs__msg__Px4ctrlDebug * ros_message = (quadrotor_msgs__msg__Px4ctrlDebug *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_v_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_v_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_v_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_v_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_v_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_v_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_v_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_v_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_v_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_a_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_a_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_a_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_a_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_a_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_a_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_a_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_a_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_a_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_a_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_a_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_a_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_a_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_a_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_a_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_a_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_a_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_a_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_q_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_q_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_q_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_q_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_q_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_q_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_q_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_q_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_q_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_q_w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_q_w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_q_w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_thr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_thr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_thr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thr2acc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thr2acc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thr2acc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // thr_scale_compensate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->thr_scale_compensate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "thr_scale_compensate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_axisang_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_axisang_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_axisang_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_axisang_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_axisang_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_axisang_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_axisang_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_axisang_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_axisang_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_axisang_ang
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_axisang_ang);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_axisang_ang", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_rate_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_rate_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_rate_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_rate_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_rate_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_rate_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fb_rate_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fb_rate_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fb_rate_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
