# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target gamesmanros_interfaces::gamesmanros_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${gamesmanros_interfaces_TARGETS}.
if(gamesmanros_interfaces_TARGETS AND NOT TARGET gamesmanros_interfaces::gamesmanros_interfaces)
  add_library(gamesmanros_interfaces::gamesmanros_interfaces INTERFACE IMPORTED)
  set_target_properties(gamesmanros_interfaces::gamesmanros_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${gamesmanros_interfaces_TARGETS}")
endif()
