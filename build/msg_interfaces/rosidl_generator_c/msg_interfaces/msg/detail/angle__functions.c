// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msg_interfaces:msg/Angle.idl
// generated code does not contain a copyright notice
#include "msg_interfaces/msg/detail/angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
msg_interfaces__msg__Angle__init(msg_interfaces__msg__Angle * msg)
{
  if (!msg) {
    return false;
  }
  // found
  // quaternion1
  // quaternion2
  // quaternion3
  // quaternion4
  // x
  // y
  // z
  return true;
}

void
msg_interfaces__msg__Angle__fini(msg_interfaces__msg__Angle * msg)
{
  if (!msg) {
    return;
  }
  // found
  // quaternion1
  // quaternion2
  // quaternion3
  // quaternion4
  // x
  // y
  // z
}

bool
msg_interfaces__msg__Angle__are_equal(const msg_interfaces__msg__Angle * lhs, const msg_interfaces__msg__Angle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // found
  if (lhs->found != rhs->found) {
    return false;
  }
  // quaternion1
  if (lhs->quaternion1 != rhs->quaternion1) {
    return false;
  }
  // quaternion2
  if (lhs->quaternion2 != rhs->quaternion2) {
    return false;
  }
  // quaternion3
  if (lhs->quaternion3 != rhs->quaternion3) {
    return false;
  }
  // quaternion4
  if (lhs->quaternion4 != rhs->quaternion4) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
msg_interfaces__msg__Angle__copy(
  const msg_interfaces__msg__Angle * input,
  msg_interfaces__msg__Angle * output)
{
  if (!input || !output) {
    return false;
  }
  // found
  output->found = input->found;
  // quaternion1
  output->quaternion1 = input->quaternion1;
  // quaternion2
  output->quaternion2 = input->quaternion2;
  // quaternion3
  output->quaternion3 = input->quaternion3;
  // quaternion4
  output->quaternion4 = input->quaternion4;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

msg_interfaces__msg__Angle *
msg_interfaces__msg__Angle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Angle * msg = (msg_interfaces__msg__Angle *)allocator.allocate(sizeof(msg_interfaces__msg__Angle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msg_interfaces__msg__Angle));
  bool success = msg_interfaces__msg__Angle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msg_interfaces__msg__Angle__destroy(msg_interfaces__msg__Angle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msg_interfaces__msg__Angle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msg_interfaces__msg__Angle__Sequence__init(msg_interfaces__msg__Angle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Angle * data = NULL;

  if (size) {
    data = (msg_interfaces__msg__Angle *)allocator.zero_allocate(size, sizeof(msg_interfaces__msg__Angle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msg_interfaces__msg__Angle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msg_interfaces__msg__Angle__fini(&data[i - 1]);
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
msg_interfaces__msg__Angle__Sequence__fini(msg_interfaces__msg__Angle__Sequence * array)
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
      msg_interfaces__msg__Angle__fini(&array->data[i]);
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

msg_interfaces__msg__Angle__Sequence *
msg_interfaces__msg__Angle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msg_interfaces__msg__Angle__Sequence * array = (msg_interfaces__msg__Angle__Sequence *)allocator.allocate(sizeof(msg_interfaces__msg__Angle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msg_interfaces__msg__Angle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msg_interfaces__msg__Angle__Sequence__destroy(msg_interfaces__msg__Angle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msg_interfaces__msg__Angle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msg_interfaces__msg__Angle__Sequence__are_equal(const msg_interfaces__msg__Angle__Sequence * lhs, const msg_interfaces__msg__Angle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msg_interfaces__msg__Angle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msg_interfaces__msg__Angle__Sequence__copy(
  const msg_interfaces__msg__Angle__Sequence * input,
  msg_interfaces__msg__Angle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msg_interfaces__msg__Angle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msg_interfaces__msg__Angle * data =
      (msg_interfaces__msg__Angle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msg_interfaces__msg__Angle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msg_interfaces__msg__Angle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msg_interfaces__msg__Angle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
