// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__CONTROLS__TRAITS_HPP_
#define MSGS_CAR__MSG__DETAIL__CONTROLS__TRAITS_HPP_

#include "msgs_car/msg/detail/controls__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'batch'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msgs_car::msg::Controls>()
{
  return "msgs_car::msg::Controls";
}

template<>
inline const char * name<msgs_car::msg::Controls>()
{
  return "msgs_car/msg/Controls";
}

template<>
struct has_fixed_size<msgs_car::msg::Controls>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseArray>::value> {};

template<>
struct has_bounded_size<msgs_car::msg::Controls>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseArray>::value> {};

template<>
struct is_message<msgs_car::msg::Controls>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS_CAR__MSG__DETAIL__CONTROLS__TRAITS_HPP_
