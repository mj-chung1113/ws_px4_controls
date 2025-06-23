// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_interfaces:msg/MarkerPoseId.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_interfaces/msg/detail/marker_pose_id__rosidl_typesupport_introspection_c.h"
#include "aruco_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_interfaces/msg/detail/marker_pose_id__functions.h"
#include "aruco_interfaces/msg/detail/marker_pose_id__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_interfaces__msg__MarkerPoseId__init(message_memory);
}

void aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_fini_function(void * message_memory)
{
  aruco_interfaces__msg__MarkerPoseId__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__MarkerPoseId, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__MarkerPoseId, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__MarkerPoseId, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_members = {
  "aruco_interfaces__msg",  // message namespace
  "MarkerPoseId",  // message name
  3,  // number of fields
  sizeof(aruco_interfaces__msg__MarkerPoseId),
  aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_member_array,  // message members
  aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_init_function,  // function to initialize message memory (memory has to be allocated)
  aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_type_support_handle = {
  0,
  &aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interfaces, msg, MarkerPoseId)() {
  aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_type_support_handle.typesupport_identifier) {
    aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aruco_interfaces__msg__MarkerPoseId__rosidl_typesupport_introspection_c__MarkerPoseId_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
