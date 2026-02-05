// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from quadrotor_msgs:msg/PolyTraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quadrotor_msgs/msg/poly_traj.h"


#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_
#define QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_

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
#include "quadrotor_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "quadrotor_msgs/msg/detail/poly_traj__struct.h"

/// Initialize msg/PolyTraj message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * quadrotor_msgs__msg__PolyTraj
 * )) before or use
 * quadrotor_msgs__msg__PolyTraj__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__init(quadrotor_msgs__msg__PolyTraj * msg);

/// Finalize msg/PolyTraj message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__PolyTraj__fini(quadrotor_msgs__msg__PolyTraj * msg);

/// Create msg/PolyTraj message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * quadrotor_msgs__msg__PolyTraj__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
quadrotor_msgs__msg__PolyTraj *
quadrotor_msgs__msg__PolyTraj__create(void);

/// Destroy msg/PolyTraj message.
/**
 * It calls
 * quadrotor_msgs__msg__PolyTraj__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__PolyTraj__destroy(quadrotor_msgs__msg__PolyTraj * msg);

/// Check for msg/PolyTraj message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__are_equal(const quadrotor_msgs__msg__PolyTraj * lhs, const quadrotor_msgs__msg__PolyTraj * rhs);

/// Copy a msg/PolyTraj message.
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
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__copy(
  const quadrotor_msgs__msg__PolyTraj * input,
  quadrotor_msgs__msg__PolyTraj * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_type_hash_t *
quadrotor_msgs__msg__PolyTraj__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_runtime_c__type_description__TypeDescription *
quadrotor_msgs__msg__PolyTraj__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_runtime_c__type_description__TypeSource *
quadrotor_msgs__msg__PolyTraj__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
quadrotor_msgs__msg__PolyTraj__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/PolyTraj messages.
/**
 * It allocates the memory for the number of elements and calls
 * quadrotor_msgs__msg__PolyTraj__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__Sequence__init(quadrotor_msgs__msg__PolyTraj__Sequence * array, size_t size);

/// Finalize array of msg/PolyTraj messages.
/**
 * It calls
 * quadrotor_msgs__msg__PolyTraj__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__PolyTraj__Sequence__fini(quadrotor_msgs__msg__PolyTraj__Sequence * array);

/// Create array of msg/PolyTraj messages.
/**
 * It allocates the memory for the array and calls
 * quadrotor_msgs__msg__PolyTraj__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
quadrotor_msgs__msg__PolyTraj__Sequence *
quadrotor_msgs__msg__PolyTraj__Sequence__create(size_t size);

/// Destroy array of msg/PolyTraj messages.
/**
 * It calls
 * quadrotor_msgs__msg__PolyTraj__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__PolyTraj__Sequence__destroy(quadrotor_msgs__msg__PolyTraj__Sequence * array);

/// Check for msg/PolyTraj message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__Sequence__are_equal(const quadrotor_msgs__msg__PolyTraj__Sequence * lhs, const quadrotor_msgs__msg__PolyTraj__Sequence * rhs);

/// Copy an array of msg/PolyTraj messages.
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
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__PolyTraj__Sequence__copy(
  const quadrotor_msgs__msg__PolyTraj__Sequence * input,
  quadrotor_msgs__msg__PolyTraj__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLY_TRAJ__FUNCTIONS_H_
