# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wander_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wander_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wander_FOUND FALSE)
  elseif(NOT wander_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wander_FOUND FALSE)
  endif()
  return()
endif()
set(_wander_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wander_FIND_QUIETLY)
  message(STATUS "Found wander: 0.0.0 (${wander_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wander' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT wander_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wander_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wander_DIR}/${_extra}")
endforeach()
