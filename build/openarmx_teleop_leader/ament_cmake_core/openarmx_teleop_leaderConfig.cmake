# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_openarmx_teleop_leader_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED openarmx_teleop_leader_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(openarmx_teleop_leader_FOUND FALSE)
  elseif(NOT openarmx_teleop_leader_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(openarmx_teleop_leader_FOUND FALSE)
  endif()
  return()
endif()
set(_openarmx_teleop_leader_CONFIG_INCLUDED TRUE)

# output package information
if(NOT openarmx_teleop_leader_FIND_QUIETLY)
  message(STATUS "Found openarmx_teleop_leader: 1.0.0 (${openarmx_teleop_leader_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'openarmx_teleop_leader' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${openarmx_teleop_leader_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(openarmx_teleop_leader_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${openarmx_teleop_leader_DIR}/${_extra}")
endforeach()
