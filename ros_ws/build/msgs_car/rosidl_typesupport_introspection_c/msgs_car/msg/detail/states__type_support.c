// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "msgs_car/msg/detail/states__rosidl_typesupport_introspection_c.h"
#include "msgs_car/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "msgs_car/msg/detail/states__functions.h"
#include "msgs_car/msg/detail/states__struct.h"


// Include directives for member types
// Member `x`
// Member `y`
// Member `vx`
// Member `vy`
// Member `psi`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void States__rosidl_typesupport_introspection_c__States_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  msgs_car__msg__States__init(message_memory);
}

void States__rosidl_typesupport_introspection_c__States_fini_function(void * message_memory)
{
  msgs_car__msg__States__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember States__rosidl_typesupport_introspection_c__States_message_member_array[6] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "psi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, psi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "psidot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(msgs_car__msg__States, psidot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers States__rosidl_typesupport_introspection_c__States_message_members = {
  "msgs_car__msg",  // message namespace
  "States",  // message name
  6,  // number of fields
  sizeof(msgs_car__msg__States),
  States__rosidl_typesupport_introspection_c__States_message_member_array,  // message members
  States__rosidl_typesupport_introspection_c__States_init_function,  // function to initialize message memory (memory has to be allocated)
  States__rosidl_typesupport_introspection_c__States_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t States__rosidl_typesupport_introspection_c__States_message_type_support_handle = {
  0,
  &States__rosidl_typesupport_introspection_c__States_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_msgs_car
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, msgs_car, msg, States)() {
  if (!States__rosidl_typesupport_introspection_c__States_message_type_support_handle.typesupport_identifier) {
    States__rosidl_typesupport_introspection_c__States_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &States__rosidl_typesupport_introspection_c__States_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
