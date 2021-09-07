// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs_car:msg/Controls.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_HPP_
#define MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'batch'
#include "geometry_msgs/msg/detail/pose_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__msgs_car__msg__Controls __attribute__((deprecated))
#else
# define DEPRECATED__msgs_car__msg__Controls __declspec(deprecated)
#endif

namespace msgs_car
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Controls_
{
  using Type = Controls_<ContainerAllocator>;

  explicit Controls_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : batch(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v = 0.0;
      this->w = 0.0;
      this->index = 0ll;
      this->goals = 0ll;
    }
  }

  explicit Controls_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : batch(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v = 0.0;
      this->w = 0.0;
      this->index = 0ll;
      this->goals = 0ll;
    }
  }

  // field types and members
  using _v_type =
    double;
  _v_type v;
  using _w_type =
    double;
  _w_type w;
  using _batch_type =
    geometry_msgs::msg::PoseArray_<ContainerAllocator>;
  _batch_type batch;
  using _index_type =
    int64_t;
  _index_type index;
  using _goals_type =
    int64_t;
  _goals_type goals;

  // setters for named parameter idiom
  Type & set__v(
    const double & _arg)
  {
    this->v = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__batch(
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> & _arg)
  {
    this->batch = _arg;
    return *this;
  }
  Type & set__index(
    const int64_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__goals(
    const int64_t & _arg)
  {
    this->goals = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs_car::msg::Controls_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs_car::msg::Controls_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs_car::msg::Controls_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs_car::msg::Controls_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs_car::msg::Controls_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs_car::msg::Controls_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs_car::msg::Controls_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs_car::msg::Controls_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs_car::msg::Controls_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs_car::msg::Controls_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs_car__msg__Controls
    std::shared_ptr<msgs_car::msg::Controls_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs_car__msg__Controls
    std::shared_ptr<msgs_car::msg::Controls_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Controls_ & other) const
  {
    if (this->v != other.v) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->batch != other.batch) {
      return false;
    }
    if (this->index != other.index) {
      return false;
    }
    if (this->goals != other.goals) {
      return false;
    }
    return true;
  }
  bool operator!=(const Controls_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Controls_

// alias to use template instance with default allocator
using Controls =
  msgs_car::msg::Controls_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs_car

#endif  // MSGS_CAR__MSG__DETAIL__CONTROLS__STRUCT_HPP_
