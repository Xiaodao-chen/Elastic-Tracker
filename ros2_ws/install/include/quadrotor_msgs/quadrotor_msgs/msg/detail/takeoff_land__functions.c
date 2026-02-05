// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/TakeoffLand.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/takeoff_land__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
quadrotor_msgs__msg__TakeoffLand__init(quadrotor_msgs__msg__TakeoffLand * msg)
{
  if (!msg) {
    return false;
  }
  // takeoff_land_cmd
  return true;
}

void
quadrotor_msgs__msg__TakeoffLand__fini(quadrotor_msgs__msg__TakeoffLand * msg)
{
  if (!msg) {
    return;
  }
  // takeoff_land_cmd
}

bool
quadrotor_msgs__msg__TakeoffLand__are_equal(const quadrotor_msgs__msg__TakeoffLand * lhs, const quadrotor_msgs__msg__TakeoffLand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // takeoff_land_cmd
  if (lhs->takeoff_land_cmd != rhs->takeoff_land_cmd) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__TakeoffLand__copy(
  const quadrotor_msgs__msg__TakeoffLand * input,
  quadrotor_msgs__msg__TakeoffLand * output)
{
  if (!input || !output) {
    return false;
  }
  // takeoff_land_cmd
  output->takeoff_land_cmd = input->takeoff_land_cmd;
  return true;
}

quadrotor_msgs__msg__TakeoffLand *
quadrotor_msgs__msg__TakeoffLand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TakeoffLand * msg = (quadrotor_msgs__msg__TakeoffLand *)allocator.allocate(sizeof(quadrotor_msgs__msg__TakeoffLand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__TakeoffLand));
  bool success = quadrotor_msgs__msg__TakeoffLand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__TakeoffLand__destroy(quadrotor_msgs__msg__TakeoffLand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__TakeoffLand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__TakeoffLand__Sequence__init(quadrotor_msgs__msg__TakeoffLand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TakeoffLand * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__TakeoffLand *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__TakeoffLand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__TakeoffLand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__TakeoffLand__fini(&data[i - 1]);
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
quadrotor_msgs__msg__TakeoffLand__Sequence__fini(quadrotor_msgs__msg__TakeoffLand__Sequence * array)
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
      quadrotor_msgs__msg__TakeoffLand__fini(&array->data[i]);
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

quadrotor_msgs__msg__TakeoffLand__Sequence *
quadrotor_msgs__msg__TakeoffLand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TakeoffLand__Sequence * array = (quadrotor_msgs__msg__TakeoffLand__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__TakeoffLand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__TakeoffLand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__TakeoffLand__Sequence__destroy(quadrotor_msgs__msg__TakeoffLand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__TakeoffLand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__TakeoffLand__Sequence__are_equal(const quadrotor_msgs__msg__TakeoffLand__Sequence * lhs, const quadrotor_msgs__msg__TakeoffLand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__TakeoffLand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__TakeoffLand__Sequence__copy(
  const quadrotor_msgs__msg__TakeoffLand__Sequence * input,
  quadrotor_msgs__msg__TakeoffLand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__TakeoffLand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__TakeoffLand * data =
      (quadrotor_msgs__msg__TakeoffLand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__TakeoffLand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__TakeoffLand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__TakeoffLand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
