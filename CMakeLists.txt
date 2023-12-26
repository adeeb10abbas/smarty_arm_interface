cmake_minimum_required(VERSION 3.5)
project(smarty_arm_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp  
  ${rosidl_generator_cpp_INCLUDE_DIRS}  
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ptipacket.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
# Declare a C++ library
add_library(${PROJECT_NAME}_src
  src/shm.c
)
ament_target_dependencies(${PROJECT_NAME}_src
  rclcpp
  std_msgs
  geometry_msgs
  Boost
  rclcpp_lifecycle
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/smarty_arm_interface.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_src
  -lrt
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  rclcpp_lifecycle
  rosidl_default_runtime
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_src
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Uncomment if the package has messages

ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "rosidl_default_runtime"
  "rclcpp_lifecycle"
  )

ament_package()
