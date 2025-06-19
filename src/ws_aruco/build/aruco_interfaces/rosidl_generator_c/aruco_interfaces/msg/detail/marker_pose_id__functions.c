// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aruco_interfaces:msg/MarkerPoseId.idl
// generated code does not contain a copyright notice
#include "aruco_interfaces/msg/detail/marker_pose_id__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
aruco_interfaces__msg__MarkerPoseId__init(aruco_interfaces__msg__MarkerPoseId * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    aruco_interfaces__msg__MarkerPoseId__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    aruco_interfaces__msg__MarkerPoseId__fini(msg);
    return false;
  }
  // id
  return true;
}

void
aruco_interfaces__msg__MarkerPoseId__fini(aruco_interfaces__msg__MarkerPoseId * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // id
}

bool
aruco_interfaces__msg__MarkerPoseId__are_equal(const aruco_interfaces__msg__MarkerPoseId * lhs, const aruco_interfaces__msg__MarkerPoseId * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
aruco_interfaces__msg__MarkerPoseId__copy(
  const aruco_interfaces__msg__MarkerPoseId * input,
  aruco_interfaces__msg__MarkerPoseId * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // id
  output->id = input->id;
  return true;
}

aruco_interfaces__msg__MarkerPoseId *
aruco_interfaces__msg__MarkerPoseId__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_interfaces__msg__MarkerPoseId * msg = (aruco_interfaces__msg__MarkerPoseId *)allocator.allocate(sizeof(aruco_interfaces__msg__MarkerPoseId), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aruco_interfaces__msg__MarkerPoseId));
  bool success = aruco_interfaces__msg__MarkerPoseId__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aruco_interfaces__msg__MarkerPoseId__destroy(aruco_interfaces__msg__MarkerPoseId * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aruco_interfaces__msg__MarkerPoseId__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aruco_interfaces__msg__MarkerPoseId__Sequence__init(aruco_interfaces__msg__MarkerPoseId__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_interfaces__msg__MarkerPoseId * data = NULL;

  if (size) {
    data = (aruco_interfaces__msg__MarkerPoseId *)allocator.zero_allocate(size, sizeof(aruco_interfaces__msg__MarkerPoseId), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aruco_interfaces__msg__MarkerPoseId__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aruco_interfaces__msg__MarkerPoseId__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
aruco_interfaces__msg__MarkerPoseId__Sequence__fini(aruco_interfaces__msg__MarkerPoseId__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      aruco_interfaces__msg__MarkerPoseId__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

aruco_interfaces__msg__MarkerPoseId__Sequence *
aruco_interfaces__msg__MarkerPoseId__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aruco_interfaces__msg__MarkerPoseId__Sequence * array = (aruco_interfaces__msg__MarkerPoseId__Sequence *)allocator.allocate(sizeof(aruco_interfaces__msg__MarkerPoseId__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aruco_interfaces__msg__MarkerPoseId__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aruco_interfaces__msg__MarkerPoseId__Sequence__destroy(aruco_interfaces__msg__MarkerPoseId__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aruco_interfaces__msg__MarkerPoseId__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aruco_interfaces__msg__MarkerPoseId__Sequence__are_equal(const aruco_interfaces__msg__MarkerPoseId__Sequence * lhs, const aruco_interfaces__msg__MarkerPoseId__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aruco_interfaces__msg__MarkerPoseId__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aruco_interfaces__msg__MarkerPoseId__Sequence__copy(
  const aruco_interfaces__msg__MarkerPoseId__Sequence * input,
  aruco_interfaces__msg__MarkerPoseId__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aruco_interfaces__msg__MarkerPoseId);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aruco_interfaces__msg__MarkerPoseId * data =
      (aruco_interfaces__msg__MarkerPoseId *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aruco_interfaces__msg__MarkerPoseId__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aruco_interfaces__msg__MarkerPoseId__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aruco_interfaces__msg__MarkerPoseId__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
