// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs_car:msg/States.idl
// generated code does not contain a copyright notice

#ifndef MSGS_CAR__MSG__DETAIL__STATES__STRUCT_HPP_
#define MSGS_CAR__MSG__DETAIL__STATES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__msgs_car__msg__States __attribute__((deprecated))
#else
# define DEPRECATED__msgs_car__msg__States __declspec(deprecated)
#endif

namespace msgs_car
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct States_
{
  using Type = States_<ContainerAllocator>;

  explicit States_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->psidot = 0.0;
    }
  }

  explicit States_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->psidot = 0.0;
    }
  }

  // field types and members
  using _x_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _x_type x;
  using _y_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _y_type y;
  using _vx_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _vx_type vx;
  using _vy_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _vy_type vy;
  using _psi_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _psi_type psi;
  using _psidot_type =
    double;
  _psidot_type psidot;

  // setters for named parameter idiom
  Type & set__x(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__vx(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__psi(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->psi = _arg;
    return *this;
  }
  Type & set__psidot(
    const double & _arg)
  {
    this->psidot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs_car::msg::States_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs_car::msg::States_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs_car::msg::States_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs_car::msg::States_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs_car::msg::States_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs_car::msg::States_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs_car::msg::States_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs_car::msg::States_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs_car::msg::States_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs_car::msg::States_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs_car__msg__States
    std::shared_ptr<msgs_car::msg::States_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs_car__msg__States
    std::shared_ptr<msgs_car::msg::States_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const States_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->psi != other.psi) {
      return false;
    }
    if (this->psidot != other.psidot) {
      return false;
    }
    return true;
  }
  bool operator!=(const States_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct States_

// alias to use template instance with default allocator
using States =
  msgs_car::msg::States_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs_car

#endif  // MSGS_CAR__MSG__DETAIL__STATES__STRUCT_HPP_
