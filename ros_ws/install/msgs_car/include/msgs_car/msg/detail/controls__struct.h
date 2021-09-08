// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_H_
#define MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'batch'
#include "geometry_msgs/msg/detail/pose_array__struct.h"

// Struct defined in msg/Controls in the package msgs_car.
typedef struct msgs_car__msg__Controls
{
  double v;
  double w;
  geometry_msgs__msg__PoseArray batch;
  int64_t index;
  int64_t goals;
} msgs_car__msg__Controls;

// Struct for a sequence of msgs_car__msg__Controls.
typedef struct msgs_car__msg__Controls__Sequence
{
  msgs_car__msg__Controls * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs_car__msg__Controls__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_H_
