# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_openarmx_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED openarmx_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(openarmx_FOUND FALSE)
  elseif(NOT openarmx_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(openarmx_FOUND FALSE)
  endif()
  return()
endif()
set(_openarmx_CONFIG_INCLUDED TRUE)

# output package information
if(NOT openarmx_FIND_QUIETLY)
  message(STATUS "Found openarmx: 1.0.0 (${openarmx_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'openarmx' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${openarmx_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(openarmx_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${openarmx_DIR}/${_extra}")
endforeach()
