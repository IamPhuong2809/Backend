cmake_minimum_required(VERSION 3.8)
project(ammr_moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake                    REQUIRED)
find_package(moveit_ros_planning_interface  REQUIRED)
find_package(moveit_task_constructor_core   REQUIRED)
find_package(rclcpp                         REQUIRED)
find_package(moveit_core                    REQUIRED)
find_package(geometry_msgs                  REQUIRED)
find_package(moveit_visual_tools            REQUIRED)
find_package(rosidl_default_generators      REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Executable: ammr_moveit_controller
add_executable(move_robot_service src/ammr_moveit_controller.cpp)
ament_target_dependencies(move_robot_service
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)
target_compile_features(move_robot_service PUBLIC c_std_99 cxx_std_17)
target_include_directories(move_robot_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_target_interfaces(move_robot_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Executable: mtc_node
add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Executable: ammr_mtc_demo
add_executable(ammr_mtc_demo src/ammr_mtc_demo.cpp)
ament_target_dependencies(ammr_mtc_demo
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
)
target_compile_features(ammr_mtc_demo PUBLIC c_std_99 cxx_std_17)
target_include_directories(ammr_mtc_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Executable: ammr_mtc_cartesian
add_executable(ammr_mtc_cartesian src/ammr_mtc_cartesian.cpp)
ament_target_dependencies(ammr_mtc_cartesian
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
)
target_compile_features(ammr_mtc_cartesian PUBLIC c_std_99 cxx_std_17)
target_include_directories(ammr_mtc_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)



# Install
install(TARGETS
  mtc_node
  ammr_mtc_demo
  ammr_mtc_cartesian
  move_robot_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
