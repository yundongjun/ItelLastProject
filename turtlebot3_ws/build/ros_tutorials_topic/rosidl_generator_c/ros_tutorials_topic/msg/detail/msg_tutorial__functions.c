// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_tutorials_topic:msg/MsgTutorial.idl
// generated code does not contain a copyright notice
#include "ros_tutorials_topic/msg/detail/msg_tutorial__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ros_tutorials_topic__msg__MsgTutorial__init(ros_tutorials_topic__msg__MsgTutorial * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    ros_tutorials_topic__msg__MsgTutorial__fini(msg);
    return false;
  }
  // data
  return true;
}

void
ros_tutorials_topic__msg__MsgTutorial__fini(ros_tutorials_topic__msg__MsgTutorial * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // data
}

bool
ros_tutorials_topic__msg__MsgTutorial__are_equal(const ros_tutorials_topic__msg__MsgTutorial * lhs, const ros_tutorials_topic__msg__MsgTutorial * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
ros_tutorials_topic__msg__MsgTutorial__copy(
  const ros_tutorials_topic__msg__MsgTutorial * input,
  ros_tutorials_topic__msg__MsgTutorial * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

ros_tutorials_topic__msg__MsgTutorial *
ros_tutorials_topic__msg__MsgTutorial__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_tutorials_topic__msg__MsgTutorial * msg = (ros_tutorials_topic__msg__MsgTutorial *)allocator.allocate(sizeof(ros_tutorials_topic__msg__MsgTutorial), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_tutorials_topic__msg__MsgTutorial));
  bool success = ros_tutorials_topic__msg__MsgTutorial__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_tutorials_topic__msg__MsgTutorial__destroy(ros_tutorials_topic__msg__MsgTutorial * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_tutorials_topic__msg__MsgTutorial__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_tutorials_topic__msg__MsgTutorial__Sequence__init(ros_tutorials_topic__msg__MsgTutorial__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_tutorials_topic__msg__MsgTutorial * data = NULL;

  if (size) {
    data = (ros_tutorials_topic__msg__MsgTutorial *)allocator.zero_allocate(size, sizeof(ros_tutorials_topic__msg__MsgTutorial), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_tutorials_topic__msg__MsgTutorial__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_tutorials_topic__msg__MsgTutorial__fini(&data[i - 1]);
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
ros_tutorials_topic__msg__MsgTutorial__Sequence__fini(ros_tutorials_topic__msg__MsgTutorial__Sequence * array)
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
      ros_tutorials_topic__msg__MsgTutorial__fini(&array->data[i]);
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

ros_tutorials_topic__msg__MsgTutorial__Sequence *
ros_tutorials_topic__msg__MsgTutorial__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_tutorials_topic__msg__MsgTutorial__Sequence * array = (ros_tutorials_topic__msg__MsgTutorial__Sequence *)allocator.allocate(sizeof(ros_tutorials_topic__msg__MsgTutorial__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_tutorials_topic__msg__MsgTutorial__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_tutorials_topic__msg__MsgTutorial__Sequence__destroy(ros_tutorials_topic__msg__MsgTutorial__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_tutorials_topic__msg__MsgTutorial__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_tutorials_topic__msg__MsgTutorial__Sequence__are_equal(const ros_tutorials_topic__msg__MsgTutorial__Sequence * lhs, const ros_tutorials_topic__msg__MsgTutorial__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_tutorials_topic__msg__MsgTutorial__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_tutorials_topic__msg__MsgTutorial__Sequence__copy(
  const ros_tutorials_topic__msg__MsgTutorial__Sequence * input,
  ros_tutorials_topic__msg__MsgTutorial__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_tutorials_topic__msg__MsgTutorial);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_tutorials_topic__msg__MsgTutorial * data =
      (ros_tutorials_topic__msg__MsgTutorial *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_tutorials_topic__msg__MsgTutorial__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_tutorials_topic__msg__MsgTutorial__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_tutorials_topic__msg__MsgTutorial__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
