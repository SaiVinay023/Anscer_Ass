cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Ensure this is included
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

# Add executable nodes
add_executable(path_publisher src/path_publisher.cpp)
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)

# Link against required libraries
ament_target_dependencies(trajectory_publisher_saver rclcpp std_msgs nav_msgs visualization_msgs tf2 tf2_ros)
ament_target_dependencies(trajectory_reader_publisher rclcpp std_msgs nav_msgs visualization_msgs tf2 tf2_ros)
ament_target_dependencies(path_publisher rclcpp nav_msgs geometry_msgs)
install(TARGETS path_publisher DESTINATION lib/${PROJECT_NAME})

# Ensure message generation is complete before building nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(trajectory_publisher_saver ${cpp_typesupport_target})
target_link_libraries(trajectory_reader_publisher ${cpp_typesupport_target})

# Install executables
install(TARGETS
  trajectory_publisher_saver
  trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install service files
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Package export
ament_export_dependencies(rosidl_default_runtime)
ament_package()
