cmake_minimum_required(VERSION 3.5)
project(bt_hhcm)

find_package(behaviortree_cpp REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(bt_hhcm SHARED 
src/cartesio_solve.cpp
src/cartesio_loader.cpp
src/cartesio_cartesian_task_control.cpp
src/robot_loader.cpp
src/robot_sense.cpp
src/robot_move.cpp
src/node_provider.cpp

)

target_link_libraries(bt_hhcm 
    PUBLIC 
    behaviortree_cpp::behaviortree_cpp 
    cartesian_interface::CartesianInterface
    rclcpp::rclcpp
    )

target_include_directories(bt_hhcm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)