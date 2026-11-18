cmake_minimum_required(VERSION 3.5)
project(ldc_navigation)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
)

# Lyrical 移除了 ament_target_dependencies；逐包映射到导出的 CMake target
set(dependency_targets
  geometry_msgs::geometry_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client
  nav_msgs::nav_msgs
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# ---- ArenaPlanner 插件库（算法在 header-only 纯 C++ 模块中） ----
add_library(arena_planner SHARED src/arena_planner.cpp)
target_include_directories(arena_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arena_planner PUBLIC ${dependency_targets})
pluginlib_export_plugin_description_file(nav2_core arena_planner_plugin.xml)

install(TARGETS arena_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# ---- 主机可跑的纯算法单测（不依赖 ROS 运行时，构建机直接执行） ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright/cpplint：本仓库无统一版权头，跳过；
  # pep257：中文文档字符串的句号（。）与 ASCII 标点规则 D213/D400/D415 硬冲突，
  #         从 lint_auto 排除后显式注册并只豁免这三条，其余照查。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(--add-ignore D213,D400,D415 test)

  add_executable(test_grid_a_star test/test_grid_a_star.cpp)
  target_include_directories(test_grid_a_star PRIVATE include)
  add_test(NAME test_grid_a_star COMMAND test_grid_a_star)

  add_executable(test_path_smoother test/test_path_smoother.cpp)
  target_include_directories(test_path_smoother PRIVATE include)
  add_test(NAME test_path_smoother COMMAND test_path_smoother)

  add_executable(test_grid_collision test/test_grid_collision.cpp)
  target_include_directories(test_grid_collision PRIVATE include)
  target_compile_options(test_grid_collision PRIVATE -UNDEBUG)
  add_test(NAME test_grid_collision COMMAND test_grid_collision)

  add_executable(test_arena_planner test/test_arena_planner.cpp)
  target_link_libraries(test_arena_planner arena_planner)
  set(arena_test_environment
    "ROS_DOMAIN_ID=194" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" "ROS_LOCALHOST_ONLY=1"
    "ROS_STATIC_PEERS=" "ROS_DISCOVERY_SERVER=" "ROS2_EASY_MODE=")
  foreach(case endpoint same_cell shared_lock snapshot cancel invalid_pose reconfigure)
    add_test(NAME arena_${case} COMMAND test_arena_planner ${case})
    set_tests_properties(arena_${case} PROPERTIES TIMEOUT 25
      ENVIRONMENT "${arena_test_environment}")
  endforeach()
  # Assertions in the existing pure tests must remain active in release builds.
  target_compile_options(test_grid_a_star PRIVATE -UNDEBUG)
  target_compile_options(test_path_smoother PRIVATE -UNDEBUG)
endif()

install(
  DIRECTORY launch config param maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
