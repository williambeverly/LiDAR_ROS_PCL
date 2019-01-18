cmake_minimum_required(VERSION 2.8.3)
project(lidar_basic)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  message_generation
)
## Generate services in 'srv' folder
add_service_files(
  FILES
  AdjustAngularVel.srv
)
## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)
## Declare a catkin package
catkin_package()
## rotate_bot
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(rotate_bot src/rotate_bot.cpp)
target_link_libraries(rotate_bot ${catkin_LIBRARIES})
add_dependencies(rotate_bot rotate_bot_generate_messages_cpp)
## process_lidar
add_executable(process_lidar src/process_lidar.cpp)
target_link_libraries(process_lidar ${catkin_LIBRARIES})
add_dependencies(process_lidar process_lidar_generate_messages_cpp)
