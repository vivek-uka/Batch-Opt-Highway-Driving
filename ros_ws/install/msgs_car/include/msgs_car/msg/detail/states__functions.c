// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice
#include "msgs_car/msg/detail/states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `x`
// Member `y`
// Member `vx`
// Member `vy`
// Member `psi`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
msgs_car__msg__States__init(msgs_car__msg__States * msg)
{
  if (!msg) {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x, 0)) {
    msgs_car__msg__States__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y, 0)) {
    msgs_car__msg__States__fini(msg);
    return false;
  }
  // vx
  if (!rosidl_runtime_c__double__Sequence__init(&msg->vx, 0)) {
    msgs_car__msg__States__fini(msg);
    return false;
  }
  // vy
  if (!rosidl_runtime_c__double__Sequence__init(&msg->vy, 0)) {
    msgs_car__msg__States__fini(msg);
    return false;
  }
  // psi
  if (!rosidl_runtime_c__double__Sequence__init(&msg->psi, 0)) {
    msgs_car__msg__States__fini(msg);
    return false;
  }
  // psidot
  return true;
}

void
msgs_car__msg__States__fini(msgs_car__msg__States * msg)
{
  if (!msg) {
    return;
  }
  // x
  rosidl_runtime_c__double__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__double__Sequence__fini(&msg->y);
  // vx
  rosidl_runtime_c__double__Sequence__fini(&msg->vx);
  // vy
  rosidl_runtime_c__double__Sequence__fini(&msg->vy);
  // psi
  rosidl_runtime_c__double__Sequence__fini(&msg->psi);
  // psidot
}

msgs_car__msg__States *
msgs_car__msg__States__create()
{
  msgs_car__msg__States * msg = (msgs_car__msg__States *)malloc(sizeof(msgs_car__msg__States));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msgs_car__msg__States));
  bool success = msgs_car__msg__States__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
msgs_car__msg__States__destroy(msgs_car__msg__States * msg)
{
  if (msg) {
    msgs_car__msg__States__fini(msg);
  }
  free(msg);
}


bool
msgs_car__msg__States__Sequence__init(msgs_car__msg__States__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  msgs_car__msg__States * data = NULL;
  if (size) {
    data = (msgs_car__msg__States *)calloc(size, sizeof(msgs_car__msg__States));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msgs_car__msg__States__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msgs_car__msg__States__fini(&data[i - 1]);
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
msgs_car__msg__States__Sequence__fini(msgs_car__msg__States__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msgs_car__msg__States__fini(&array->data[i]);
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

msgs_car__msg__States__Sequence *
msgs_car__msg__States__Sequence__create(size_t size)
{
  msgs_car__msg__States__Sequence * array = (msgs_car__msg__States__Sequence *)malloc(sizeof(msgs_car__msg__States__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = msgs_car__msg__States__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
msgs_car__msg__States__Sequence__destroy(msgs_car__msg__States__Sequence * array)
{
  if (array) {
    msgs_car__msg__States__Sequence__fini(array);
  }
  free(array);
}
