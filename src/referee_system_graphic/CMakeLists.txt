cmake_minimum_required(VERSION 3.8)
project(referee_system_graphic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_msg_interface REQUIRED)

add_executable(referee_system_graphic
  src/referee_system_graphic.cpp
  src/MappingTables.cpp
  src/DataType.cpp
)

ament_target_dependencies(referee_system_graphic
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
  my_msg_interface

)
target_include_directories(referee_system_graphic
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/referee_system_graphic/
)
install(TARGETS 
referee_system_graphic
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
launch DESTINATION 
share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
