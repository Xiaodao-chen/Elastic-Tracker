// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from decomp_ros_msgs:msg/Polyhedron.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "decomp_ros_msgs/msg/polyhedron.h"


#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__FUNCTIONS_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "decomp_ros_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "decomp_ros_msgs/msg/detail/polyhedron__struct.h"

/// Initialize msg/Polyhedron message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * decomp_ros_msgs__msg__Polyhedron
 * )) before or use
 * decomp_ros_msgs__msg__Polyhedron__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__Polyhedron__init(decomp_ros_msgs__msg__Polyhedron * msg);

/// Finalize msg/Polyhedron message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__Polyhedron__fini(decomp_ros_msgs__msg__Polyhedron * msg);

/// Create msg/Polyhedron message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * decomp_ros_msgs__msg__Polyhedron__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
decomp_ros_msgs__msg__Polyhedron *
decomp_ros_msgs__msg__Polyhedron__create(void);

/// Destroy msg/Polyhedron message.
/**
 * It calls
 * decomp_ros_msgs__msg__Polyhedron__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__Polyhedron__destroy(decomp_ros_msgs__msg__Polyhedron * msg);

/// Check for msg/Polyhedron message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__Polyhedron__are_equal(const decomp_ros_msgs__msg__Polyhedron * lhs, const decomp_ros_msgs__msg__Polyhedron * rhs);

/// Copy a msg/Polyhedron message.
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
decomp_ros_msgs__msg__Polyhedron__copy(
  const decomp_ros_msgs__msg__Polyhedron * input,
  decomp_ros_msgs__msg__Polyhedron * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_type_hash_t *
decomp_ros_msgs__msg__Polyhedron__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_runtime_c__type_description__TypeDescription *
decomp_ros_msgs__msg__Polyhedron__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_runtime_c__type_description__TypeSource *
decomp_ros_msgs__msg__Polyhedron__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
decomp_ros_msgs__msg__Polyhedron__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/Polyhedron messages.
/**
 * It allocates the memory for the number of elements and calls
 * decomp_ros_msgs__msg__Polyhedron__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__Polyhedron__Sequence__init(decomp_ros_msgs__msg__Polyhedron__Sequence * array, size_t size);

/// Finalize array of msg/Polyhedron messages.
/**
 * It calls
 * decomp_ros_msgs__msg__Polyhedron__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__Polyhedron__Sequence__fini(decomp_ros_msgs__msg__Polyhedron__Sequence * array);

/// Create array of msg/Polyhedron messages.
/**
 * It allocates the memory for the array and calls
 * decomp_ros_msgs__msg__Polyhedron__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
decomp_ros_msgs__msg__Polyhedron__Sequence *
decomp_ros_msgs__msg__Polyhedron__Sequence__create(size_t size);

/// Destroy array of msg/Polyhedron messages.
/**
 * It calls
 * decomp_ros_msgs__msg__Polyhedron__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
void
decomp_ros_msgs__msg__Polyhedron__Sequence__destroy(decomp_ros_msgs__msg__Polyhedron__Sequence * array);

/// Check for msg/Polyhedron message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_decomp_ros_msgs
bool
decomp_ros_msgs__msg__Polyhedron__Sequence__are_equal(const decomp_ros_msgs__msg__Polyhedron__Sequence * lhs, const decomp_ros_msgs__msg__Polyhedron__Sequence * rhs);

/// Copy an array of msg/Polyhedron messages.
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
decomp_ros_msgs__msg__Polyhedron__Sequence__copy(
  const decomp_ros_msgs__msg__Polyhedron__Sequence * input,
  decomp_ros_msgs__msg__Polyhedron__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON__FUNCTIONS_H_
