// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aruco_interfaces:msg/MarkerPoseId.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_H_
#define ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/MarkerPoseId in the package aruco_interfaces.
typedef struct aruco_interfaces__msg__MarkerPoseId
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  int32_t id;
} aruco_interfaces__msg__MarkerPoseId;

// Struct for a sequence of aruco_interfaces__msg__MarkerPoseId.
typedef struct aruco_interfaces__msg__MarkerPoseId__Sequence
{
  aruco_interfaces__msg__MarkerPoseId * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aruco_interfaces__msg__MarkerPoseId__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_H_
