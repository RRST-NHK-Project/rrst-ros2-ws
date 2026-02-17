# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nr26_r2_hw_ctrl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nr26_r2_hw_ctrl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nr26_r2_hw_ctrl_FOUND FALSE)
  elseif(NOT nr26_r2_hw_ctrl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nr26_r2_hw_ctrl_FOUND FALSE)
  endif()
  return()
endif()
set(_nr26_r2_hw_ctrl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nr26_r2_hw_ctrl_FIND_QUIETLY)
  message(STATUS "Found nr26_r2_hw_ctrl: 0.0.0 (${nr26_r2_hw_ctrl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nr26_r2_hw_ctrl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT nr26_r2_hw_ctrl_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nr26_r2_hw_ctrl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nr26_r2_hw_ctrl_DIR}/${_extra}")
endforeach()
