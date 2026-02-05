// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from decomp_ros_msgs:msg/EllipsoidArray.idl
// generated code does not contain a copyright notice
#include "decomp_ros_msgs/msg/detail/ellipsoid_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ellipsoids`
#include "decomp_ros_msgs/msg/detail/ellipsoid__functions.h"

bool
decomp_ros_msgs__msg__EllipsoidArray__init(decomp_ros_msgs__msg__EllipsoidArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    decomp_ros_msgs__msg__EllipsoidArray__fini(msg);
    return false;
  }
  // ellipsoids
  if (!decomp_ros_msgs__msg__Ellipsoid__Sequence__init(&msg->ellipsoids, 0)) {
    decomp_ros_msgs__msg__EllipsoidArray__fini(msg);
    return false;
  }
  return true;
}

void
decomp_ros_msgs__msg__EllipsoidArray__fini(decomp_ros_msgs__msg__EllipsoidArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // ellipsoids
  decomp_ros_msgs__msg__Ellipsoid__Sequence__fini(&msg->ellipsoids);
}

bool
decomp_ros_msgs__msg__EllipsoidArray__are_equal(const decomp_ros_msgs__msg__EllipsoidArray * lhs, const decomp_ros_msgs__msg__EllipsoidArray * rhs)
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
  // ellipsoids
  if (!decomp_ros_msgs__msg__Ellipsoid__Sequence__are_equal(
      &(lhs->ellipsoids), &(rhs->ellipsoids)))
  {
    return false;
  }
  return true;
}

bool
decomp_ros_msgs__msg__EllipsoidArray__copy(
  const decomp_ros_msgs__msg__EllipsoidArray * input,
  decomp_ros_msgs__msg__EllipsoidArray * output)
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
  // ellipsoids
  if (!decomp_ros_msgs__msg__Ellipsoid__Sequence__copy(
      &(input->ellipsoids), &(output->ellipsoids)))
  {
    return false;
  }
  return true;
}

decomp_ros_msgs__msg__EllipsoidArray *
decomp_ros_msgs__msg__EllipsoidArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  decomp_ros_msgs__msg__EllipsoidArray * msg = (decomp_ros_msgs__msg__EllipsoidArray *)allocator.allocate(sizeof(decomp_ros_msgs__msg__EllipsoidArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(decomp_ros_msgs__msg__EllipsoidArray));
  bool success = decomp_ros_msgs__msg__EllipsoidArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
decomp_ros_msgs__msg__EllipsoidArray__destroy(decomp_ros_msgs__msg__EllipsoidArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    decomp_ros_msgs__msg__EllipsoidArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
decomp_ros_msgs__msg__EllipsoidArray__Sequence__init(decomp_ros_msgs__msg__EllipsoidArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  decomp_ros_msgs__msg__EllipsoidArray * data = NULL;

  if (size) {
    data = (decomp_ros_msgs__msg__EllipsoidArray *)allocator.zero_allocate(size, sizeof(decomp_ros_msgs__msg__EllipsoidArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = decomp_ros_msgs__msg__EllipsoidArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        decomp_ros_msgs__msg__EllipsoidArray__fini(&data[i - 1]);
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
decomp_ros_msgs__msg__EllipsoidArray__Sequence__fini(decomp_ros_msgs__msg__EllipsoidArray__Sequence * array)
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
      decomp_ros_msgs__msg__EllipsoidArray__fini(&array->data[i]);
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

decomp_ros_msgs__msg__EllipsoidArray__Sequence *
decomp_ros_msgs__msg__EllipsoidArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  decomp_ros_msgs__msg__EllipsoidArray__Sequence * array = (decomp_ros_msgs__msg__EllipsoidArray__Sequence *)allocator.allocate(sizeof(decomp_ros_msgs__msg__EllipsoidArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = decomp_ros_msgs__msg__EllipsoidArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
decomp_ros_msgs__msg__EllipsoidArray__Sequence__destroy(decomp_ros_msgs__msg__EllipsoidArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    decomp_ros_msgs__msg__EllipsoidArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
decomp_ros_msgs__msg__EllipsoidArray__Sequence__are_equal(const decomp_ros_msgs__msg__EllipsoidArray__Sequence * lhs, const decomp_ros_msgs__msg__EllipsoidArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!decomp_ros_msgs__msg__EllipsoidArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
decomp_ros_msgs__msg__EllipsoidArray__Sequence__copy(
  const decomp_ros_msgs__msg__EllipsoidArray__Sequence * input,
  decomp_ros_msgs__msg__EllipsoidArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(decomp_ros_msgs__msg__EllipsoidArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    decomp_ros_msgs__msg__EllipsoidArray * data =
      (decomp_ros_msgs__msg__EllipsoidArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!decomp_ros_msgs__msg__EllipsoidArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          decomp_ros_msgs__msg__EllipsoidArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!decomp_ros_msgs__msg__EllipsoidArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
