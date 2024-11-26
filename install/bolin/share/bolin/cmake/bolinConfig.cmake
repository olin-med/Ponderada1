# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bolin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bolin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bolin_FOUND FALSE)
  elseif(NOT bolin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bolin_FOUND FALSE)
  endif()
  return()
endif()
set(_bolin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bolin_FIND_QUIETLY)
  message(STATUS "Found bolin: 0.1.0 (${bolin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bolin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bolin_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bolin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bolin_DIR}/${_extra}")
endforeach()
