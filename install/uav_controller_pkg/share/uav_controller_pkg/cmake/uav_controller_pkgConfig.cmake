# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_uav_controller_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED uav_controller_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(uav_controller_pkg_FOUND FALSE)
  elseif(NOT uav_controller_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(uav_controller_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_uav_controller_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT uav_controller_pkg_FIND_QUIETLY)
  message(STATUS "Found uav_controller_pkg: 0.0.0 (${uav_controller_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'uav_controller_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${uav_controller_pkg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(uav_controller_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${uav_controller_pkg_DIR}/${_extra}")
endforeach()
