cmake_minimum_required(VERSION 3.8)
project(piper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# include directories
include_directories(include)

# Install Python launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install SDF / STL files
install(DIRECTORY objects
  DESTINATION share/${PROJECT_NAME}/
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()