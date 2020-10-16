// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from navatics_msgs:msg/CanBus.idl
// generated code does not contain a copyright notice
#include "navatics_msgs/msg/can_bus__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `data`
#include "rosidl_generator_c/primitives_sequence_functions.h"

bool
navatics_msgs__msg__CanBus__init(navatics_msgs__msg__CanBus * msg)
{
  if (!msg) {
    return false;
  }
  // std_id
  // length
  // data
  if (!rosidl_generator_c__uint8__Sequence__init(&msg->data, 0)) {
    navatics_msgs__msg__CanBus__fini(msg);
    return false;
  }
  return true;
}

void
navatics_msgs__msg__CanBus__fini(navatics_msgs__msg__CanBus * msg)
{
  if (!msg) {
    return;
  }
  // std_id
  // length
  // data
  rosidl_generator_c__uint8__Sequence__fini(&msg->data);
}

navatics_msgs__msg__CanBus *
navatics_msgs__msg__CanBus__create()
{
  navatics_msgs__msg__CanBus * msg = (navatics_msgs__msg__CanBus *)malloc(sizeof(navatics_msgs__msg__CanBus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(navatics_msgs__msg__CanBus));
  bool success = navatics_msgs__msg__CanBus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
navatics_msgs__msg__CanBus__destroy(navatics_msgs__msg__CanBus * msg)
{
  if (msg) {
    navatics_msgs__msg__CanBus__fini(msg);
  }
  free(msg);
}


bool
navatics_msgs__msg__CanBus__Sequence__init(navatics_msgs__msg__CanBus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  navatics_msgs__msg__CanBus * data = NULL;
  if (size) {
    data = (navatics_msgs__msg__CanBus *)calloc(size, sizeof(navatics_msgs__msg__CanBus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = navatics_msgs__msg__CanBus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        navatics_msgs__msg__CanBus__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
navatics_msgs__msg__CanBus__Sequence__fini(navatics_msgs__msg__CanBus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      navatics_msgs__msg__CanBus__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

navatics_msgs__msg__CanBus__Sequence *
navatics_msgs__msg__CanBus__Sequence__create(size_t size)
{
  navatics_msgs__msg__CanBus__Sequence * array = (navatics_msgs__msg__CanBus__Sequence *)malloc(sizeof(navatics_msgs__msg__CanBus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = navatics_msgs__msg__CanBus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
navatics_msgs__msg__CanBus__Sequence__destroy(navatics_msgs__msg__CanBus__Sequence * array)
{
  if (array) {
    navatics_msgs__msg__CanBus__Sequence__fini(array);
  }
  free(array);
}
