cmake_minimum_required(VERSION 3.8)
project(quadrupled_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IKService.srv"
 )

 
add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller rosidl_default_runtime trajectory_msgs rclcpp std_msgs kdl_parser tf2_eigen_kdl orocos_kdl_vendor)
rosidl_target_interfaces(robot_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(ik_service src/ik_service.cpp)
ament_target_dependencies(ik_service rosidl_default_runtime rclcpp std_msgs kdl_parser tf2_eigen_kdl orocos_kdl_vendor)
rosidl_target_interfaces(ik_service ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/path_follower.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ik_service
  robot_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
