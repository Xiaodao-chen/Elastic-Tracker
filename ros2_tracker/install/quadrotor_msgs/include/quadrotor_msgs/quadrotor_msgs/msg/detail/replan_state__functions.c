// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/ReplanState.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/replan_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ini_state`
// Member `path`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `target`
#include "nav_msgs/msg/detail/odometry__functions.h"
// Member `occmap`
#include "quadrotor_msgs/msg/detail/occ_map3d__functions.h"
// Member `replan_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
quadrotor_msgs__msg__ReplanState__init(quadrotor_msgs__msg__ReplanState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  // state
  // ini_state
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ini_state, 0)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  // target
  if (!nav_msgs__msg__Odometry__init(&msg->target)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  // occmap
  if (!quadrotor_msgs__msg__OccMap3d__init(&msg->occmap)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  // path
  if (!rosidl_runtime_c__double__Sequence__init(&msg->path, 0)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  // replan_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->replan_stamp)) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
    return false;
  }
  return true;
}

void
quadrotor_msgs__msg__ReplanState__fini(quadrotor_msgs__msg__ReplanState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  // ini_state
  rosidl_runtime_c__double__Sequence__fini(&msg->ini_state);
  // target
  nav_msgs__msg__Odometry__fini(&msg->target);
  // occmap
  quadrotor_msgs__msg__OccMap3d__fini(&msg->occmap);
  // path
  rosidl_runtime_c__double__Sequence__fini(&msg->path);
  // replan_stamp
  builtin_interfaces__msg__Time__fini(&msg->replan_stamp);
}

bool
quadrotor_msgs__msg__ReplanState__are_equal(const quadrotor_msgs__msg__ReplanState * lhs, const quadrotor_msgs__msg__ReplanState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // ini_state
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ini_state), &(rhs->ini_state)))
  {
    return false;
  }
  // target
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  // occmap
  if (!quadrotor_msgs__msg__OccMap3d__are_equal(
      &(lhs->occmap), &(rhs->occmap)))
  {
    return false;
  }
  // path
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  // replan_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->replan_stamp), &(rhs->replan_stamp)))
  {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__ReplanState__copy(
  const quadrotor_msgs__msg__ReplanState * input,
  quadrotor_msgs__msg__ReplanState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // state
  output->state = input->state;
  // ini_state
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ini_state), &(output->ini_state)))
  {
    return false;
  }
  // target
  if (!nav_msgs__msg__Odometry__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  // occmap
  if (!quadrotor_msgs__msg__OccMap3d__copy(
      &(input->occmap), &(output->occmap)))
  {
    return false;
  }
  // path
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  // replan_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->replan_stamp), &(output->replan_stamp)))
  {
    return false;
  }
  return true;
}

quadrotor_msgs__msg__ReplanState *
quadrotor_msgs__msg__ReplanState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__ReplanState * msg = (quadrotor_msgs__msg__ReplanState *)allocator.allocate(sizeof(quadrotor_msgs__msg__ReplanState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__ReplanState));
  bool success = quadrotor_msgs__msg__ReplanState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__ReplanState__destroy(quadrotor_msgs__msg__ReplanState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__ReplanState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__ReplanState__Sequence__init(quadrotor_msgs__msg__ReplanState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__ReplanState * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__ReplanState *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__ReplanState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__ReplanState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__ReplanState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
quadrotor_msgs__msg__ReplanState__Sequence__fini(quadrotor_msgs__msg__ReplanState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      quadrotor_msgs__msg__ReplanState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

quadrotor_msgs__msg__ReplanState__Sequence *
quadrotor_msgs__msg__ReplanState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__ReplanState__Sequence * array = (quadrotor_msgs__msg__ReplanState__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__ReplanState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__ReplanState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__ReplanState__Sequence__destroy(quadrotor_msgs__msg__ReplanState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__ReplanState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__ReplanState__Sequence__are_equal(const quadrotor_msgs__msg__ReplanState__Sequence * lhs, const quadrotor_msgs__msg__ReplanState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__ReplanState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__ReplanState__Sequence__copy(
  const quadrotor_msgs__msg__ReplanState__Sequence * input,
  quadrotor_msgs__msg__ReplanState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__ReplanState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__ReplanState * data =
      (quadrotor_msgs__msg__ReplanState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__ReplanState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__ReplanState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__ReplanState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
