# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_socket_server_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED socket_server_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(socket_server_pkg_FOUND FALSE)
  elseif(NOT socket_server_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(socket_server_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_socket_server_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT socket_server_pkg_FIND_QUIETLY)
  message(STATUS "Found socket_server_pkg: 0.0.0 (${socket_server_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'socket_server_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${socket_server_pkg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(socket_server_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${socket_server_pkg_DIR}/${_extra}")
endforeach()
