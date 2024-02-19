// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from commsmsgs:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef COMMSMSGS__MSG__DETAIL__SPHERE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define COMMSMSGS__MSG__DETAIL__SPHERE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "commsmsgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "commsmsgs/msg/detail/sphere__struct.hpp"

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

namespace commsmsgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_commsmsgs
cdr_serialize(
  const commsmsgs::msg::Sphere & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_commsmsgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  commsmsgs::msg::Sphere & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_commsmsgs
get_serialized_size(
  const commsmsgs::msg::Sphere & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_commsmsgs
max_serialized_size_Sphere(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace commsmsgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_commsmsgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, commsmsgs, msg, Sphere)();

#ifdef __cplusplus
}
#endif

#endif  // COMMSMSGS__MSG__DETAIL__SPHERE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
