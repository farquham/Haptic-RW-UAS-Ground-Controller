// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from commsmsgs:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef COMMSMSGS__MSG__DETAIL__NUM__STRUCT_H_
#define COMMSMSGS__MSG__DETAIL__NUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Num in the package commsmsgs.
typedef struct commsmsgs__msg__Num
{
  int64_t num;
} commsmsgs__msg__Num;

// Struct for a sequence of commsmsgs__msg__Num.
typedef struct commsmsgs__msg__Num__Sequence
{
  commsmsgs__msg__Num * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} commsmsgs__msg__Num__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMSMSGS__MSG__DETAIL__NUM__STRUCT_H_
