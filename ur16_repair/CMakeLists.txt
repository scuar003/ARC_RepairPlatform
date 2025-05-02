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

include_directories(
  include
)

add_library(repair_menu src/main_menu.cpp)
target_include_directories(repair_menu PUBLIC include)
ament_target_dependencies( repair_menu visualization_msgs interactive_markers)

#set dependecies
set(dependencies
  repair_interface
  rclcpp
  visualization_msgs
  interactive_markers
  )

add_executable(supervisor src/supervisor.cpp)
target_link_libraries(supervisor repair_menu)
ament_target_dependencies(supervisor ${dependencies})

install(TARGETS
  supervisor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
