cmake_minimum_required(VERSION 3.8)
project(ldc_qt_gui)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Gui Core)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(SOURCES
  include/ldc_qt_gui/main_window.hpp
  include/ldc_qt_gui/map_view.hpp
  include/ldc_qt_gui/lidar_view.hpp
  include/ldc_qt_gui/camera_view.hpp
  src/main.cpp
  src/main_window.cpp
  src/ros_interface.cpp
  src/map_view.cpp
  src/lidar_view.cpp
  src/camera_view.cpp
)

add_executable(ldc_qt_gui ${SOURCES})
target_include_directories(ldc_qt_gui PRIVATE include)
target_link_libraries(ldc_qt_gui Qt5::Widgets Qt5::Gui Qt5::Core)
target_link_libraries(ldc_qt_gui
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
)

install(TARGETS ldc_qt_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
