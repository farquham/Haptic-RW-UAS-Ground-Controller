// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from commsmsgs:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef COMMSMSGS__MSG__DETAIL__NUM__BUILDER_HPP_
#define COMMSMSGS__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "commsmsgs/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace commsmsgs
{

namespace msg
{

namespace builder
{

class Init_Num_num
{
public:
  Init_Num_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::commsmsgs::msg::Num num(::commsmsgs::msg::Num::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::commsmsgs::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::commsmsgs::msg::Num>()
{
  return commsmsgs::msg::builder::Init_Num_num();
}

}  // namespace commsmsgs

#endif  // COMMSMSGS__MSG__DETAIL__NUM__BUILDER_HPP_
