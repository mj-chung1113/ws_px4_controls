// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from aruco_interfaces:msg/MarkerPoseId.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_HPP_
#define ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__aruco_interfaces__msg__MarkerPoseId __attribute__((deprecated))
#else
# define DEPRECATED__aruco_interfaces__msg__MarkerPoseId __declspec(deprecated)
#endif

namespace aruco_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarkerPoseId_
{
  using Type = MarkerPoseId_<ContainerAllocator>;

  explicit MarkerPoseId_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
    }
  }

  explicit MarkerPoseId_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _id_type =
    int32_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> *;
  using ConstRawPtr =
    const aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__aruco_interfaces__msg__MarkerPoseId
    std::shared_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__aruco_interfaces__msg__MarkerPoseId
    std::shared_ptr<aruco_interfaces::msg::MarkerPoseId_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarkerPoseId_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarkerPoseId_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarkerPoseId_

// alias to use template instance with default allocator
using MarkerPoseId =
  aruco_interfaces::msg::MarkerPoseId_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace aruco_interfaces

#endif  // ARUCO_INTERFACES__MSG__DETAIL__MARKER_POSE_ID__STRUCT_HPP_
