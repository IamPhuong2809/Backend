cmake_minimum_required(VERSION 3.8)
project(ammr_moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(ammr_moveit_controller src/ammr_moveit_controller.cpp)
ament_target_dependencies(
  ammr_moveit_controller
  rclcpp
  moveit_visual_tools
  moveit_ros_planning_interface
)
target_compile_features(ammr_moveit_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ammr_moveit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)
add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(
  mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(ammr_mtc_demo src/ammr_mtc_demo.cpp)
ament_target_dependencies(
  ammr_mtc_demo
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
)
target_compile_features(ammr_mtc_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17




install(TARGETS 
  ammr_moveit_controller
  mtc_node
  ammr_mtc_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()