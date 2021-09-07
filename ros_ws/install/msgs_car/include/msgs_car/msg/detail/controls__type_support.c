// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msgs_car/msg/detail/controls__rosidl_typesupport_introspection_c.h"
#include "msgs_car/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msgs_car/msg/detail/controls__functions.h"
#include "msgs_car/msg/detail/controls__struct.h"


// Include directives for member types
// Member `batch`
#include "geometry_msgs/msg/pose_array.h"
// Member `batch`
#include "geometry_msgs/msg/detail/pose_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Controls__rosidl_typesupport_introspection_c__Controls_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msgs_car__msg__Controls__init(message_memory);
}

void Controls__rosidl_typesupport_introspection_c__Controls_fini_function(void * message_memory)
{
  msgs_car__msg__Controls__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Controls__rosidl_typesupport_introspection_c__Controls_message_member_array[5] = {
  {
    "v",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__Controls, v),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "w",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__Controls, w),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "batch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__Controls, batch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__Controls, index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goals",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__Controls, goals),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Controls__rosidl_typesupport_introspection_c__Controls_message_members = {
  "msgs_car__msg",  // message namespace
  "Controls",  // message name
  5,  // number of fields
  sizeof(msgs_car__msg__Controls),
  Controls__rosidl_typesupport_introspection_c__Controls_message_member_array,  // message members
  Controls__rosidl_typesupport_introspection_c__Controls_init_function,  // function to initialize message memory (memory has to be allocated)
  Controls__rosidl_typesupport_introspection_c__Controls_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Controls__rosidl_typesupport_introspection_c__Controls_message_type_support_handle = {
  0,
  &Controls__rosidl_typesupport_introspection_c__Controls_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msgs_car
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msgs_car, msg, Controls)() {
  Controls__rosidl_typesupport_introspection_c__Controls_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseArray)();
  if (!Controls__rosidl_typesupport_introspection_c__Controls_message_type_support_handle.typesupport_identifier) {
    Controls__rosidl_typesupport_introspection_c__Controls_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Controls__rosidl_typesupport_introspection_c__Controls_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
