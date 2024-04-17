cmake_minimum_required(VERSION 3.1)
project(exchangestation_detector)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_STANDARD 14) 


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(serial_DIR "${CMAKE_INSTALL_PREFIX}/serial/share/serial/cmake")
set(msg_interfaces_DIR "${CMAKE_INSTALL_PREFIX}/msg_interfaces/share/msg_interfaces")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(serial REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()



# 创建库文件目标
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  cv_bridge
  serial
  msg_interfaces
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${image_transport_INCLUDE_DIRS}
)




# 注册节点组件

find_package(rclcpp_components REQUIRED)

RCLCPP_COMPONENTS_REGISTER_NODE(${PROJECT_NAME} 
  PLUGIN exchangestation_detector_node
  EXECUTABLE detector_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

