// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from object_detection_msgs:msg/MotorAngle.idl
// generated code does not contain a copyright notice
#include "object_detection_msgs/msg/detail/motor_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
object_detection_msgs__msg__MotorAngle__init(object_detection_msgs__msg__MotorAngle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    object_detection_msgs__msg__MotorAngle__fini(msg);
    return false;
  }
  // angle
  return true;
}

void
object_detection_msgs__msg__MotorAngle__fini(object_detection_msgs__msg__MotorAngle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // angle
}

bool
object_detection_msgs__msg__MotorAngle__are_equal(const object_detection_msgs__msg__MotorAngle * lhs, const object_detection_msgs__msg__MotorAngle * rhs)
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
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  return true;
}

bool
object_detection_msgs__msg__MotorAngle__copy(
  const object_detection_msgs__msg__MotorAngle * input,
  object_detection_msgs__msg__MotorAngle * output)
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
  // angle
  output->angle = input->angle;
  return true;
}

object_detection_msgs__msg__MotorAngle *
object_detection_msgs__msg__MotorAngle__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  object_detection_msgs__msg__MotorAngle * msg = (object_detection_msgs__msg__MotorAngle *)allocator.allocate(sizeof(object_detection_msgs__msg__MotorAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(object_detection_msgs__msg__MotorAngle));
  bool success = object_detection_msgs__msg__MotorAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
object_detection_msgs__msg__MotorAngle__destroy(object_detection_msgs__msg__MotorAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    object_detection_msgs__msg__MotorAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
object_detection_msgs__msg__MotorAngle__Sequence__init(object_detection_msgs__msg__MotorAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  object_detection_msgs__msg__MotorAngle * data = NULL;

  if (size) {
    data = (object_detection_msgs__msg__MotorAngle *)allocator.zero_allocate(size, sizeof(object_detection_msgs__msg__MotorAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = object_detection_msgs__msg__MotorAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        object_detection_msgs__msg__MotorAngle__fini(&data[i - 1]);
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
object_detection_msgs__msg__MotorAngle__Sequence__fini(object_detection_msgs__msg__MotorAngle__Sequence * array)
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
      object_detection_msgs__msg__MotorAngle__fini(&array->data[i]);
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

object_detection_msgs__msg__MotorAngle__Sequence *
object_detection_msgs__msg__MotorAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  object_detection_msgs__msg__MotorAngle__Sequence * array = (object_detection_msgs__msg__MotorAngle__Sequence *)allocator.allocate(sizeof(object_detection_msgs__msg__MotorAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = object_detection_msgs__msg__MotorAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
object_detection_msgs__msg__MotorAngle__Sequence__destroy(object_detection_msgs__msg__MotorAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    object_detection_msgs__msg__MotorAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
object_detection_msgs__msg__MotorAngle__Sequence__are_equal(const object_detection_msgs__msg__MotorAngle__Sequence * lhs, const object_detection_msgs__msg__MotorAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!object_detection_msgs__msg__MotorAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
object_detection_msgs__msg__MotorAngle__Sequence__copy(
  const object_detection_msgs__msg__MotorAngle__Sequence * input,
  object_detection_msgs__msg__MotorAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(object_detection_msgs__msg__MotorAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    object_detection_msgs__msg__MotorAngle * data =
      (object_detection_msgs__msg__MotorAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!object_detection_msgs__msg__MotorAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          object_detection_msgs__msg__MotorAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!object_detection_msgs__msg__MotorAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
