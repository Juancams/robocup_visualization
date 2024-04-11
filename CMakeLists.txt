cmake_minimum_required(VERSION 3.5)
project(robocup_visualization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)

set(robocup_visualization_headers_to_moc
  include/robocup_visualization/RobocupPanel.hpp
)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/RobocupPanel.cpp
  ${robocup_visualization_headers_to_moc}
)

set(dependencies
  pluginlib
  Qt5
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  map_msgs
  rclcpp
  coresense_instrumentation_interfaces
  lifecycle_msgs
)

ament_package()

