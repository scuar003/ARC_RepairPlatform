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
find_package(tf2_eigen REQUIRED)
find_package(Open3D REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters octree)
find_package(std_srvs REQUIRED)




set(UR_RTDE_DIR "/home/robotics/p_devel/libs_/ur_rtde")
set(UR_RTDE_INCLUDE_DIR "${UR_RTDE_DIR}/include")
set(UR_RTDE_LIBRARY "${UR_RTDE_DIR}/build/librtde.so")

include_directories(
  include
  ${UR_RTDE_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Open3D_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

## ----------------------- Local Libraries ---------------- #
set(repair_helpers
  src/repair/repair_operations.cpp 
  src/repair/surface_detection.cpp 

)
add_library(repair_menu src/ui/main_menu.cpp)
target_include_directories(repair_menu PUBLIC include)
ament_target_dependencies( repair_menu visualization_msgs interactive_markers)


add_library(repair_operations ${repair_helpers})
target_include_directories(repair_operations PUBLIC include)
target_link_libraries(repair_operations ${UR_RTDE_LIBRARY} ur16_geometry)
ament_target_dependencies(repair_operations rclcpp repair_interface sensor_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs Open3D )

#devel not fully tested
add_library(ur16_geometry src/repair/plane_geometry.cpp)
target_include_directories(ur16_geometry PUBLIC include)
target_link_libraries(ur16_geometry Eigen3::Eigen)
ament_target_dependencies(ur16_geometry geometry_msgs)

#set dependecies
set(dependencies
  repair_interface
  rclcpp
  rclcpp_action
  visualization_msgs
  interactive_markers
  std_srvs
  )


### ----------------- Nodes and Executables -------------------- ###
#supervisor
add_executable(supervisor src/supervisor.cpp)
target_link_libraries(supervisor repair_menu)
ament_target_dependencies(supervisor ${dependencies})

#repair server 
add_executable(repair_server src/repair/repair_server.cpp)
target_link_libraries(repair_server repair_operations)
ament_target_dependencies(repair_server ${dependencies})

#renderer
add_executable(renderer src/ui/renderer.cpp)
target_link_libraries(renderer repair_operations ur16_geometry)
ament_target_dependencies(renderer ${dependencies})

#laser to cloud node 
add_executable(laser_to_cloud src/cloud/laser_to_cloud.cpp)
ament_target_dependencies(laser_to_cloud rclcpp sensor_msgs  pcl_conversions tf2_ros  tf2_eigen)
target_link_libraries(laser_to_cloud ${PCL_LIBRARIES}  Eigen3::Eigen)

add_executable(voxel_mapper src/cloud/laser_map.cpp)
ament_target_dependencies(voxel_mapper rclcpp sensor_msgs tf2_ros tf2_eigen pcl_conversions std_srvs)
target_link_libraries(voxel_mapper ${PCL_INCLUDE_DIRS})




install(TARGETS
  supervisor
  repair_server
  renderer
  laser_to_cloud
  voxel_mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
