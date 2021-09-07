// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__STATES__STRUCT_H_
#define MSGS_CAR__MSG__DETAIL__STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'x'
// Member 'y'
// Member 'vx'
// Member 'vy'
// Member 'psi'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/States in the package msgs_car.
typedef struct msgs_car__msg__States
{
  rosidl_runtime_c__double__Sequence x;
  rosidl_runtime_c__double__Sequence y;
  rosidl_runtime_c__double__Sequence vx;
  rosidl_runtime_c__double__Sequence vy;
  rosidl_runtime_c__double__Sequence psi;
  double psidot;
} msgs_car__msg__States;

// Struct for a sequence of msgs_car__msg__States.
typedef struct msgs_car__msg__States__Sequence
{
  msgs_car__msg__States * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs_car__msg__States__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS_CAR__MSG__DETAIL__STATES__STRUCT_H_
