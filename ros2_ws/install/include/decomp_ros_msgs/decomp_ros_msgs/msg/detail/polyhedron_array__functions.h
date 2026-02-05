// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__FUNCTIONS_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "decomp_ros_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "decomp_ros_msgs/msg/detail/polyhedron_array__struct.h"

/// Initialize msg/PolyhedronArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * decomp_ros_msgs__msg__PolyhedronArray
 * )) before or use
 * decomp_ros_msgs__msg__PolyhedronArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__init(decomp_ros_msgs__msg__PolyhedronArray * msg);

/// Finalize msg/PolyhedronArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__PolyhedronArray__fini(decomp_ros_msgs__msg__PolyhedronArray * msg);

/// Create msg/PolyhedronArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * decomp_ros_msgs__msg__PolyhedronArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
decomp_ros_msgs__msg__PolyhedronArray *
decomp_ros_msgs__msg__PolyhedronArray__create();

/// Destroy msg/PolyhedronArray message.
/**
 * It calls
 * decomp_ros_msgs__msg__PolyhedronArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__PolyhedronArray__destroy(decomp_ros_msgs__msg__PolyhedronArray * msg);

/// Check for msg/PolyhedronArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__are_equal(const decomp_ros_msgs__msg__PolyhedronArray * lhs, const decomp_ros_msgs__msg__PolyhedronArray * rhs);

/// Copy a msg/PolyhedronArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__copy(
  const decomp_ros_msgs__msg__PolyhedronArray * input,
  decomp_ros_msgs__msg__PolyhedronArray * output);

/// Initialize array of msg/PolyhedronArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * decomp_ros_msgs__msg__PolyhedronArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__Sequence__init(decomp_ros_msgs__msg__PolyhedronArray__Sequence * array, size_t size);

/// Finalize array of msg/PolyhedronArray messages.
/**
 * It calls
 * decomp_ros_msgs__msg__PolyhedronArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__PolyhedronArray__Sequence__fini(decomp_ros_msgs__msg__PolyhedronArray__Sequence * array);

/// Create array of msg/PolyhedronArray messages.
/**
 * It allocates the memory for the array and calls
 * decomp_ros_msgs__msg__PolyhedronArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
decomp_ros_msgs__msg__PolyhedronArray__Sequence *
decomp_ros_msgs__msg__PolyhedronArray__Sequence__create(size_t size);

/// Destroy array of msg/PolyhedronArray messages.
/**
 * It calls
 * decomp_ros_msgs__msg__PolyhedronArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__PolyhedronArray__Sequence__destroy(decomp_ros_msgs__msg__PolyhedronArray__Sequence * array);

/// Check for msg/PolyhedronArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__Sequence__are_equal(const decomp_ros_msgs__msg__PolyhedronArray__Sequence * lhs, const decomp_ros_msgs__msg__PolyhedronArray__Sequence * rhs);

/// Copy an array of msg/PolyhedronArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__PolyhedronArray__Sequence__copy(
  const decomp_ros_msgs__msg__PolyhedronArray__Sequence * input,
  decomp_ros_msgs__msg__PolyhedronArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__FUNCTIONS_H_
