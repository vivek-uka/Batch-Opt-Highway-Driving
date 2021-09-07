// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__CONTROLS__BUILDER_HPP_
#define MSGS_CAR__MSG__DETAIL__CONTROLS__BUILDER_HPP_

#include "msgs_car/msg/detail/controls__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace msgs_car
{

namespace msg
{

namespace builder
{

class Init_Controls_goals
{
public:
  explicit Init_Controls_goals(::msgs_car::msg::Controls & msg)
  : msg_(msg)
  {}
  ::msgs_car::msg::Controls goals(::msgs_car::msg::Controls::_goals_type arg)
  {
    msg_.goals = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs_car::msg::Controls msg_;
};

class Init_Controls_index
{
public:
  explicit Init_Controls_index(::msgs_car::msg::Controls & msg)
  : msg_(msg)
  {}
  Init_Controls_goals index(::msgs_car::msg::Controls::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_Controls_goals(msg_);
  }

private:
  ::msgs_car::msg::Controls msg_;
};

class Init_Controls_batch
{
public:
  explicit Init_Controls_batch(::msgs_car::msg::Controls & msg)
  : msg_(msg)
  {}
  Init_Controls_index batch(::msgs_car::msg::Controls::_batch_type arg)
  {
    msg_.batch = std::move(arg);
    return Init_Controls_index(msg_);
  }

private:
  ::msgs_car::msg::Controls msg_;
};

class Init_Controls_w
{
public:
  explicit Init_Controls_w(::msgs_car::msg::Controls & msg)
  : msg_(msg)
  {}
  Init_Controls_batch w(::msgs_car::msg::Controls::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Controls_batch(msg_);
  }

private:
  ::msgs_car::msg::Controls msg_;
};

class Init_Controls_v
{
public:
  Init_Controls_v()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Controls_w v(::msgs_car::msg::Controls::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_Controls_w(msg_);
  }

private:
  ::msgs_car::msg::Controls msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs_car::msg::Controls>()
{
  return msgs_car::msg::builder::Init_Controls_v();
}

}  // namespace msgs_car

#endif  // MSGS_CAR__MSG__DETAIL__CONTROLS__BUILDER_HPP_
