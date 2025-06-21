cmake_minimum_required(VERSION 3.16)
project(my_robot_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)


# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

#find_package(libslmp2 REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  my_robot_hw
  SHARED
  hardware/ammr_system.cpp
)

target_compile_features(my_robot_hw PUBLIC cxx_std_17)

target_include_directories(my_robot_hw PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/my_robot_hw>
  /usr/local/include
  /usr/local/include/melcli
)

target_link_libraries(my_robot_hw PUBLIC melcli)


ament_target_dependencies(
  my_robot_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)



# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface my_robot_hw.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/my_robot_hw
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/meshes
  DESTINATION share/my_robot_hw
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/my_robot_hw
)
install(TARGETS my_robot_hw
  EXPORT export_my_robot_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_my_robot_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
