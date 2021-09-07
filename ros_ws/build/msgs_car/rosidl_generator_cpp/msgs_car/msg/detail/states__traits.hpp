// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__STATES__TRAITS_HPP_
#define MSGS_CAR__MSG__DETAIL__STATES__TRAITS_HPP_

#include "msgs_car/msg/detail/states__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<msgs_car::msg::States>()
{
  return "msgs_car::msg::States";
}

template<>
inline const char * name<msgs_car::msg::States>()
{
  return "msgs_car/msg/States";
}

template<>
struct has_fixed_size<msgs_car::msg::States>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msgs_car::msg::States>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msgs_car::msg::States>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSGS_CAR__MSG__DETAIL__STATES__TRAITS_HPP_
