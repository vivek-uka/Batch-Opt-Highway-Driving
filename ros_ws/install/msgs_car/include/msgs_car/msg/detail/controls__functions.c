// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice
#include "msgs_car/msg/detail/controls__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `batch`
#include "geometry_msgs/msg/detail/pose_array__functions.h"

bool
msgs_car__msg__Controls__init(msgs_car__msg__Controls * msg)
{
  if (!msg) {
    return false;
  }
  // v
  // w
  // batch
  if (!geometry_msgs__msg__PoseArray__init(&msg->batch)) {
    msgs_car__msg__Controls__fini(msg);
    return false;
  }
  // index
  // goals
  return true;
}

void
msgs_car__msg__Controls__fini(msgs_car__msg__Controls * msg)
{
  if (!msg) {
    return;
  }
  // v
  // w
  // batch
  geometry_msgs__msg__PoseArray__fini(&msg->batch);
  // index
  // goals
}

msgs_car__msg__Controls *
msgs_car__msg__Controls__create()
{
  msgs_car__msg__Controls * msg = (msgs_car__msg__Controls *)malloc(sizeof(msgs_car__msg__Controls));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msgs_car__msg__Controls));
  bool success = msgs_car__msg__Controls__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
msgs_car__msg__Controls__destroy(msgs_car__msg__Controls * msg)
{
  if (msg) {
    msgs_car__msg__Controls__fini(msg);
  }
  free(msg);
}


bool
msgs_car__msg__Controls__Sequence__init(msgs_car__msg__Controls__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  msgs_car__msg__Controls * data = NULL;
  if (size) {
    data = (msgs_car__msg__Controls *)calloc(size, sizeof(msgs_car__msg__Controls));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msgs_car__msg__Controls__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msgs_car__msg__Controls__fini(&data[i - 1]);
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
msgs_car__msg__Controls__Sequence__fini(msgs_car__msg__Controls__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msgs_car__msg__Controls__fini(&array->data[i]);
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

msgs_car__msg__Controls__Sequence *
msgs_car__msg__Controls__Sequence__create(size_t size)
{
  msgs_car__msg__Controls__Sequence * array = (msgs_car__msg__Controls__Sequence *)malloc(sizeof(msgs_car__msg__Controls__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = msgs_car__msg__Controls__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
msgs_car__msg__Controls__Sequence__destroy(msgs_car__msg__Controls__Sequence * array)
{
  if (array) {
    msgs_car__msg__Controls__Sequence__fini(array);
  }
  free(array);
}
