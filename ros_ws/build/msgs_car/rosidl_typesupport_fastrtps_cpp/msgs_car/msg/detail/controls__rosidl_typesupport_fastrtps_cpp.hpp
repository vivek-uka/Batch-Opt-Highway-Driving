// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__CONTROLS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MSGS_CAR__MSG__DETAIL__CONTROLS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "msgs_car/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "msgs_car/msg/detail/controls__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace msgs_car
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_car
cdr_serialize(
  const msgs_car::msg::Controls & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_car
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  msgs_car::msg::Controls & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_car
get_serialized_size(
  const msgs_car::msg::Controls & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_car
max_serialized_size_Controls(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace msgs_car

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_msgs_car
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, msgs_car, msg, Controls)();

#ifdef __cplusplus
}
#endif

#endif  // MSGS_CAR__MSG__DETAIL__CONTROLS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
