# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rescue_orchestrator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rescue_orchestrator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rescue_orchestrator_FOUND FALSE)
  elseif(NOT rescue_orchestrator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rescue_orchestrator_FOUND FALSE)
  endif()
  return()
endif()
set(_rescue_orchestrator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rescue_orchestrator_FIND_QUIETLY)
  message(STATUS "Found rescue_orchestrator: 0.0.1 (${rescue_orchestrator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rescue_orchestrator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rescue_orchestrator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rescue_orchestrator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rescue_orchestrator_DIR}/${_extra}")
endforeach()
