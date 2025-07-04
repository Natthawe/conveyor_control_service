cmake_minimum_required(VERSION 3.8)
project(conveyor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
 
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConveyorCommand.srv"
)

add_executable(conveyor_service_node src/conveyor_service_node.cpp)
ament_target_dependencies(conveyor_service_node rclcpp)

add_executable(conveyor_client_node src/conveyor_client_node.cpp)
ament_target_dependencies(conveyor_client_node rclcpp)

rosidl_target_interfaces(conveyor_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(conveyor_client_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  conveyor_service_node
  conveyor_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
