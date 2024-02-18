# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rpicomms_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rpicomms_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rpicomms_FOUND FALSE)
  elseif(NOT rpicomms_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rpicomms_FOUND FALSE)
  endif()
  return()
endif()
set(_rpicomms_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rpicomms_FIND_QUIETLY)
  message(STATUS "Found rpicomms: 0.0.0 (${rpicomms_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rpicomms' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rpicomms_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rpicomms_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rpicomms_DIR}/${_extra}")
endforeach()
