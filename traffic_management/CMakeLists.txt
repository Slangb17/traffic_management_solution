cmake_minimum_required(VERSION 3.8)
project(traffic_management)

set(lib_name ${PROJECT_NAME})

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

set(dep_pkgs
    rclcpp
    std_msgs
    nav2_costmap_2d
    nav2_map_server
    pluginlib
    rclcpp_action
    rclcpp_lifecycle
    visualization_msgs
    nav2_util
    nav2_msgs
    nav_msgs
    geometry_msgs
    builtin_interfaces
    tf2_ros
    nav2_core
    OpenCV
)

# === Build ===


add_library(${lib_name} SHARED
  src/traffic_lanes.cpp
  src/restricted_areas.cpp
  src/tm_navfn_planner.cpp
  src/tm_navfn.cpp
)
include_directories(include)


# Install nodes:
install(TARGETS ${lib_name}
  DESTINATION lib)

# Install directories: launch, params, urdf, worlds
install(DIRECTORY
  launch
  worlds
  models
  maps
  config
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}/
)



# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d traffic_lanes.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d  restricted_areas.xml)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()