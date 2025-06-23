// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aruco_interfaces:msg/MarkerPoseId.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__BUILDER_HPP_
#define ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "aruco_interfaces/msg/detail/marker_pose_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace aruco_interfaces
{

namespace msg
{

namespace builder
{

class Init_MarkerPoseId_id
{
public:
  explicit Init_MarkerPoseId_id(::aruco_interfaces::msg::MarkerPoseId & msg)
  : msg_(msg)
  {}
  ::aruco_interfaces::msg::MarkerPoseId id(::aruco_interfaces::msg::MarkerPoseId::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_interfaces::msg::MarkerPoseId msg_;
};

class Init_MarkerPoseId_pose
{
public:
  explicit Init_MarkerPoseId_pose(::aruco_interfaces::msg::MarkerPoseId & msg)
  : msg_(msg)
  {}
  Init_MarkerPoseId_id pose(::aruco_interfaces::msg::MarkerPoseId::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_MarkerPoseId_id(msg_);
  }

private:
  ::aruco_interfaces::msg::MarkerPoseId msg_;
};

class Init_MarkerPoseId_header
{
public:
  Init_MarkerPoseId_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarkerPoseId_pose header(::aruco_interfaces::msg::MarkerPoseId::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MarkerPoseId_pose(msg_);
  }

private:
  ::aruco_interfaces::msg::MarkerPoseId msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_interfaces::msg::MarkerPoseId>()
{
  return aruco_interfaces::msg::builder::Init_MarkerPoseId_header();
}

}  // namespace aruco_interfaces

#endif  // ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__BUILDER_HPP_
