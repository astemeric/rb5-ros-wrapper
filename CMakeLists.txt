cmake_minimum_required(VERSION 2.8.3)
project(rb5_ros_wrapper)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  message_generation
)

add_message_files(
  FILES rx.msg
)

add_action_files(
  DIRECTORY action
  FILES Motion.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(rb5_ros_wrapper src/wrapper.cpp src/lanros2podo.cpp)
target_link_libraries(rb5_ros_wrapper ${catkin_LIBRARIES})
add_dependencies(rb5_ros_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rb5_update src/update.cpp src/lanpodo2ros.cpp)
target_link_libraries(rb5_update ${catkin_LIBRARIES})
add_dependencies(rb5_update ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(RB5_Client src/RB5_Client.cpp)
target_link_libraries(RB5_Client ${catkin_LIBRARIES})
add_dependencies(RB5_Client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(cobot_controller src/main.cpp)
target_link_libraries(cobot_controller ${catkin_LIBRARIES})
add_dependencies(cobot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
)
