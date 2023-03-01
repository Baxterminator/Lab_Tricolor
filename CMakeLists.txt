cmake_minimum_required(VERSION 3.5)
project(lab_tricolor)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})
#add_definitions(${EIGEN3_DEFINITIONS})
#find_package(orocos_kdl REQUIRED)
#target_link_libraries(${PROJECT_NAME} orocos_kdl)

#rosidl_generate_interfaces(${PROJECT_NAME} "srv/Jacobian.srv")
#ament_export_dependencies(rosidl_default_runtime)

#if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
#    set(LEGACY_IDL TRUE)
#else()
#    set(LEGACY_IDL FALSE)
#endif()

#if(NOT ${LEGACY_IDL})
#    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
#endif()

#macro(add_message node_name)
#    if(${LEGACY_IDL})
#        rosidl_target_interfaces(${node_name} "${PROJECT_NAME}" "rosidl_typesupport_cpp")
#    else()
#        target_link_libraries(${node_name} "${cpp_typesupport_target}")
#    endif()
#endmacro()

ament_auto_add_executable(sort_ball src/sort_ball.cpp)
target_include_directories(sort_ball PRIVATE include)

ament_auto_add_executable(jacobian src/jacobian.cpp)
target_include_directories(jacobian PRIVATE include)
#add_message(jacobian)

target_link_libraries(jacobian orocos_kdl)


ament_package()
