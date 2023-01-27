cmake_minimum_required(VERSION 3.5)
project(lab_tricolor)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sort_ball
        src/sort_ball.cpp)
target_include_directories(sort_ball
        PRIVATE include)
ament_target_dependencies(sort_ball
        rclcpp)

install(TARGETS sort_ball
        DESTINATION lib/${PROJECT_NAME})

ament_package()
