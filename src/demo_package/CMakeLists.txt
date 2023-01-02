cmake_minimum_required(VERSION 3.8)
project(demo_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(EEROS REQUIRED)
find_package(ros-eeros REQUIRED)

add_compile_definitions(USE_ROS)

set(EEROS_INCLUDE_DIRS "/home/lukas/Documents/Projects/vt/src/eeros-project/install-x86/include/")

list(APPEND MSG_LIBRARIES ${std_msgs_LIBRARIES})
list(APPEND MSG_LIBRARIES ${sensor_msgs_LIBRARIES})
list(APPEND MSG_LIBRARIES ${geometry_msgs_LIBRARIES})

add_executable(${PROJECT_NAME} src/demo_package.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp)
ament_target_dependencies(${PROJECT_NAME} std_msgs)
ament_target_dependencies(${PROJECT_NAME} sensor_msgs)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs)

link_directories(${ros-eeros_DIR})
target_link_libraries(${PROJECT_NAME} eeros ${ros-eeros_DIR}/libroseeros.so)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EEROS_INCLUDE_DIRS}
)

# Install launch files
install(
  DIRECTORY launch description
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION bin/${PROJECT_NAME}
)

ament_package()
