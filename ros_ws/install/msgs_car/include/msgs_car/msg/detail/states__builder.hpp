// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__STATES__BUILDER_HPP_
#define MSGS_CAR__MSG__DETAIL__STATES__BUILDER_HPP_

#include "msgs_car/msg/detail/states__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace msgs_car
{

namespace msg
{

namespace builder
{

class Init_States_psidot
{
public:
  explicit Init_States_psidot(::msgs_car::msg::States & msg)
  : msg_(msg)
  {}
  ::msgs_car::msg::States psidot(::msgs_car::msg::States::_psidot_type arg)
  {
    msg_.psidot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

class Init_States_psi
{
public:
  explicit Init_States_psi(::msgs_car::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_psidot psi(::msgs_car::msg::States::_psi_type arg)
  {
    msg_.psi = std::move(arg);
    return Init_States_psidot(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

class Init_States_vy
{
public:
  explicit Init_States_vy(::msgs_car::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_psi vy(::msgs_car::msg::States::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_States_psi(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

class Init_States_vx
{
public:
  explicit Init_States_vx(::msgs_car::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_vy vx(::msgs_car::msg::States::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_States_vy(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

class Init_States_y
{
public:
  explicit Init_States_y(::msgs_car::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_vx y(::msgs_car::msg::States::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_States_vx(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

class Init_States_x
{
public:
  Init_States_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_States_y x(::msgs_car::msg::States::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_States_y(msg_);
  }

private:
  ::msgs_car::msg::States msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs_car::msg::States>()
{
  return msgs_car::msg::builder::Init_States_x();
}

}  // namespace msgs_car

#endif  // MSGS_CAR__MSG__DETAIL__STATES__BUILDER_HPP_
