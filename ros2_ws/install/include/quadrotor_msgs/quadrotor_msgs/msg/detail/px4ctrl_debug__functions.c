// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/px4ctrl_debug__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
quadrotor_msgs__msg__Px4ctrlDebug__init(quadrotor_msgs__msg__Px4ctrlDebug * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__Px4ctrlDebug__fini(msg);
    return false;
  }
  // des_v_x
  // des_v_y
  // des_v_z
  // fb_a_x
  // fb_a_y
  // fb_a_z
  // des_a_x
  // des_a_y
  // des_a_z
  // des_q_x
  // des_q_y
  // des_q_z
  // des_q_w
  // des_thr
  // thr2acc
  // thr_scale_compensate
  // voltage
  // err_axisang_x
  // err_axisang_y
  // err_axisang_z
  // err_axisang_ang
  // fb_rate_x
  // fb_rate_y
  // fb_rate_z
  return true;
}

void
quadrotor_msgs__msg__Px4ctrlDebug__fini(quadrotor_msgs__msg__Px4ctrlDebug * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // des_v_x
  // des_v_y
  // des_v_z
  // fb_a_x
  // fb_a_y
  // fb_a_z
  // des_a_x
  // des_a_y
  // des_a_z
  // des_q_x
  // des_q_y
  // des_q_z
  // des_q_w
  // des_thr
  // thr2acc
  // thr_scale_compensate
  // voltage
  // err_axisang_x
  // err_axisang_y
  // err_axisang_z
  // err_axisang_ang
  // fb_rate_x
  // fb_rate_y
  // fb_rate_z
}

bool
quadrotor_msgs__msg__Px4ctrlDebug__are_equal(const quadrotor_msgs__msg__Px4ctrlDebug * lhs, const quadrotor_msgs__msg__Px4ctrlDebug * rhs)
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
  // des_v_x
  if (lhs->des_v_x != rhs->des_v_x) {
    return false;
  }
  // des_v_y
  if (lhs->des_v_y != rhs->des_v_y) {
    return false;
  }
  // des_v_z
  if (lhs->des_v_z != rhs->des_v_z) {
    return false;
  }
  // fb_a_x
  if (lhs->fb_a_x != rhs->fb_a_x) {
    return false;
  }
  // fb_a_y
  if (lhs->fb_a_y != rhs->fb_a_y) {
    return false;
  }
  // fb_a_z
  if (lhs->fb_a_z != rhs->fb_a_z) {
    return false;
  }
  // des_a_x
  if (lhs->des_a_x != rhs->des_a_x) {
    return false;
  }
  // des_a_y
  if (lhs->des_a_y != rhs->des_a_y) {
    return false;
  }
  // des_a_z
  if (lhs->des_a_z != rhs->des_a_z) {
    return false;
  }
  // des_q_x
  if (lhs->des_q_x != rhs->des_q_x) {
    return false;
  }
  // des_q_y
  if (lhs->des_q_y != rhs->des_q_y) {
    return false;
  }
  // des_q_z
  if (lhs->des_q_z != rhs->des_q_z) {
    return false;
  }
  // des_q_w
  if (lhs->des_q_w != rhs->des_q_w) {
    return false;
  }
  // des_thr
  if (lhs->des_thr != rhs->des_thr) {
    return false;
  }
  // thr2acc
  if (lhs->thr2acc != rhs->thr2acc) {
    return false;
  }
  // thr_scale_compensate
  if (lhs->thr_scale_compensate != rhs->thr_scale_compensate) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // err_axisang_x
  if (lhs->err_axisang_x != rhs->err_axisang_x) {
    return false;
  }
  // err_axisang_y
  if (lhs->err_axisang_y != rhs->err_axisang_y) {
    return false;
  }
  // err_axisang_z
  if (lhs->err_axisang_z != rhs->err_axisang_z) {
    return false;
  }
  // err_axisang_ang
  if (lhs->err_axisang_ang != rhs->err_axisang_ang) {
    return false;
  }
  // fb_rate_x
  if (lhs->fb_rate_x != rhs->fb_rate_x) {
    return false;
  }
  // fb_rate_y
  if (lhs->fb_rate_y != rhs->fb_rate_y) {
    return false;
  }
  // fb_rate_z
  if (lhs->fb_rate_z != rhs->fb_rate_z) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__Px4ctrlDebug__copy(
  const quadrotor_msgs__msg__Px4ctrlDebug * input,
  quadrotor_msgs__msg__Px4ctrlDebug * output)
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
  // des_v_x
  output->des_v_x = input->des_v_x;
  // des_v_y
  output->des_v_y = input->des_v_y;
  // des_v_z
  output->des_v_z = input->des_v_z;
  // fb_a_x
  output->fb_a_x = input->fb_a_x;
  // fb_a_y
  output->fb_a_y = input->fb_a_y;
  // fb_a_z
  output->fb_a_z = input->fb_a_z;
  // des_a_x
  output->des_a_x = input->des_a_x;
  // des_a_y
  output->des_a_y = input->des_a_y;
  // des_a_z
  output->des_a_z = input->des_a_z;
  // des_q_x
  output->des_q_x = input->des_q_x;
  // des_q_y
  output->des_q_y = input->des_q_y;
  // des_q_z
  output->des_q_z = input->des_q_z;
  // des_q_w
  output->des_q_w = input->des_q_w;
  // des_thr
  output->des_thr = input->des_thr;
  // thr2acc
  output->thr2acc = input->thr2acc;
  // thr_scale_compensate
  output->thr_scale_compensate = input->thr_scale_compensate;
  // voltage
  output->voltage = input->voltage;
  // err_axisang_x
  output->err_axisang_x = input->err_axisang_x;
  // err_axisang_y
  output->err_axisang_y = input->err_axisang_y;
  // err_axisang_z
  output->err_axisang_z = input->err_axisang_z;
  // err_axisang_ang
  output->err_axisang_ang = input->err_axisang_ang;
  // fb_rate_x
  output->fb_rate_x = input->fb_rate_x;
  // fb_rate_y
  output->fb_rate_y = input->fb_rate_y;
  // fb_rate_z
  output->fb_rate_z = input->fb_rate_z;
  return true;
}

quadrotor_msgs__msg__Px4ctrlDebug *
quadrotor_msgs__msg__Px4ctrlDebug__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Px4ctrlDebug * msg = (quadrotor_msgs__msg__Px4ctrlDebug *)allocator.allocate(sizeof(quadrotor_msgs__msg__Px4ctrlDebug), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__Px4ctrlDebug));
  bool success = quadrotor_msgs__msg__Px4ctrlDebug__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__Px4ctrlDebug__destroy(quadrotor_msgs__msg__Px4ctrlDebug * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__Px4ctrlDebug__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__init(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Px4ctrlDebug * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__Px4ctrlDebug *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__Px4ctrlDebug), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__Px4ctrlDebug__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__Px4ctrlDebug__fini(&data[i - 1]);
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
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__fini(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array)
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
      quadrotor_msgs__msg__Px4ctrlDebug__fini(&array->data[i]);
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

quadrotor_msgs__msg__Px4ctrlDebug__Sequence *
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array = (quadrotor_msgs__msg__Px4ctrlDebug__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__Px4ctrlDebug__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__Px4ctrlDebug__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__destroy(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__Px4ctrlDebug__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__are_equal(const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * lhs, const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__Px4ctrlDebug__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__copy(
  const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * input,
  quadrotor_msgs__msg__Px4ctrlDebug__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__Px4ctrlDebug);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__Px4ctrlDebug * data =
      (quadrotor_msgs__msg__Px4ctrlDebug *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__Px4ctrlDebug__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__Px4ctrlDebug__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__Px4ctrlDebug__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
