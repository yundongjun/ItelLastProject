# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:ros_goal_client__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:ros_goal_client__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:ros_goal_client__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:ros_goal_client__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:ros_goal_client__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:ros_goal_client__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:ros_goal_client__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:ros_goal_client__rosidl_typesupport_cpp;__rosidl_generator_py:ros_goal_client__rosidl_generator_py")

# populate ros_goal_client_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "ros_goal_client::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'ros_goal_client' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND ros_goal_client_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
