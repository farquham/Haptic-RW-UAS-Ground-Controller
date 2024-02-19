// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from commsmsgs:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef COMMSMSGS__MSG__DETAIL__SPHERE__BUILDER_HPP_
#define COMMSMSGS__MSG__DETAIL__SPHERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "commsmsgs/msg/detail/sphere__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace commsmsgs
{

namespace msg
{

namespace builder
{

class Init_Sphere_radius
{
public:
  explicit Init_Sphere_radius(::commsmsgs::msg::Sphere & msg)
  : msg_(msg)
  {}
  ::commsmsgs::msg::Sphere radius(::commsmsgs::msg::Sphere::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::commsmsgs::msg::Sphere msg_;
};

class Init_Sphere_center
{
public:
  Init_Sphere_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sphere_radius center(::commsmsgs::msg::Sphere::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_Sphere_radius(msg_);
  }

private:
  ::commsmsgs::msg::Sphere msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::commsmsgs::msg::Sphere>()
{
  return commsmsgs::msg::builder::Init_Sphere_center();
}

}  // namespace commsmsgs

#endif  // COMMSMSGS__MSG__DETAIL__SPHERE__BUILDER_HPP_
