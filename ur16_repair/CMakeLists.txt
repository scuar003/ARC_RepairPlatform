cmake_minimum_required(VERSION 3.8)
project(ur16_repair)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependecies 
find_package(ament_cmake REQUIRED)
find_package(repair_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Open3D REQUIRED)


set(UR_RTDE_DIR "/home/robotics/p_devel/libs_/ur_rtde")
set(UR_RTDE_INCLUDE_DIR "${UR_RTDE_DIR}/include")
set(UR_RTDE_LIBRARY "${UR_RTDE_DIR}/build/librtde.so")

include_directories(
  include
  ${UR_RTDE_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

link_directories(${Open3D_LIBRARY_DIRS})

add_library(repair_menu src/ui/main_menu.cpp)
target_include_directories(repair_menu PUBLIC include)
ament_target_dependencies( repair_menu visualization_msgs interactive_markers)

#devel not fully tested
add_library(repair_operations src/repair/repair_operations.cpp src/repair/surface_detection.cpp)
target_include_directories(repair_operations PUBLIC include)
target_link_libraries(repair_operations ${UR_RTDE_LIBRARY})
ament_target_dependencies(repair_operations rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs Open3D )



#set dependecies
set(dependencies
  repair_interface
  rclcpp
  rclcpp_action
  visualization_msgs
  interactive_markers
  )

#supervisor
add_executable(supervisor src/supervisor.cpp)
target_link_libraries(supervisor repair_menu)
ament_target_dependencies(supervisor ${dependencies})

#repair server 
add_executable(repair_server src/repair/repair_server.cpp)
target_link_libraries(repair_server repair_operations)
ament_target_dependencies(repair_server ${dependencies})

install(TARGETS
  supervisor
  repair_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
