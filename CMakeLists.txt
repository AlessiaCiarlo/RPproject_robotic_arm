cmake_minimum_required(VERSION 3.8)
project(robotic_arm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(ik_node src/robot_invkinematics.cpp)
target_link_libraries(ik_node yaml-cpp)

ament_target_dependencies(ik_node rclcpp std_msgs sensor_msgs Eigen3 yaml-cpp)

install(TARGETS ik_node DESTINATION lib/${PROJECT_NAME})
        
# Copy necessary files to designated locations in the project
install (DIRECTORY config launch rviz urdf src DESTINATION share/${PROJECT_NAME})

 
# Automates the process of setting up linting for the package, which
# is the process of running tools that analyze the code for potential
# errors, style issues, and other discrepancies that do not adhere to
# specified coding standards or best practices.
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
 
ament_package()
