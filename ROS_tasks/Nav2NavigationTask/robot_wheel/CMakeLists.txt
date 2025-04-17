cmake_minimum_required(VERSION 3.8)
project(robot_wheel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_messsages_interface)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rovorMove src/RovorMovemain.cpp src/rovorMove.cpp)
add_executable(imuDat src/imuMain.cpp src/imuStuff.cpp)
add_executable(getDestination src/getDestination.cpp)
add_executable(logData src/logMain.cpp)

ament_target_dependencies(rovorMove rclcpp geometry_msgs sensor_msgs custom_messsages_interface)
ament_target_dependencies(imuDat rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs custom_messsages_interface)
ament_target_dependencies(getDestination rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs custom_messsages_interface)
ament_target_dependencies(logData rclcpp sensor_msgs geometry_msgs)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS rovorMove
  TARGETS imuDat
  TARGETS getDestination
  TARGETS logData
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
