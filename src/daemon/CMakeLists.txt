cmake_minimum_required(VERSION 3.8)
project(muto_daemon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(muto_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nlohmann_json 3 REQUIRED)

# Library with all daemon components
add_library(muto_daemon_lib
  src/actual_state_prober.cpp
  src/desired_state_resolver.cpp
  src/drift_detector.cpp
  src/graph_state_publisher.cpp
  src/muto_daemon_node.cpp
)
target_include_directories(muto_daemon_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(muto_daemon_lib
  rclcpp
  muto_msgs
  std_msgs
  builtin_interfaces
)
target_link_libraries(muto_daemon_lib nlohmann_json::nlohmann_json)

# Executable
add_executable(muto_daemon src/main.cpp)
target_link_libraries(muto_daemon muto_daemon_lib)

# Install
install(TARGETS muto_daemon
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS muto_daemon_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_drift_detector test/test_drift_detector.cpp)
  target_link_libraries(test_drift_detector muto_daemon_lib)

  ament_add_gtest(test_desired_state_resolver test/test_desired_state_resolver.cpp)
  target_link_libraries(test_desired_state_resolver muto_daemon_lib)

  add_launch_test(test/integration/test_daemon_observation.py)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
