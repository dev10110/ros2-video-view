cmake_minimum_required(VERSION 3.16)
project(video_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  # set(CMAKE_BUILD_TYPE "Debug")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

set(node_plugins "")

add_library(video_view_component SHARED src/video_viewer.cpp)

target_compile_definitions(video_view_component
                           PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(video_view_component ${OpenCV_LIBS})
target_precompile_headers(video_view_component PRIVATE
                          include/video_view/precompile.hpp)
ament_target_dependencies(video_view_component rclcpp rclcpp_components
                          sensor_msgs image_transport cv_bridge)

rclcpp_components_register_node(
  video_view_component PLUGIN "video_view::VideoViewer" EXECUTABLE
  video_viewer_node)

set(node_plugins
    "${node_plugins}video_view::VideoViewer;$<TARGET_FILE:video_viewer>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS video_view_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)

ament_package()
