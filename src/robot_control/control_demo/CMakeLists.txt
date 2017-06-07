cmake_minimum_required(VERSION 2.8.3)
project(control_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS kdl_parser roscpp arobot_whisper arobot_hw controller_manager arobot_kdl arobot_db
            actionlib control_msgs trajectory_msgs
        )
find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS arobot_whisper arobot_hw controller_manager  kdl_parser roscpp arobot_kdl arobot_db 
  DEPENDS orocos_kdl
)

set(HIREDIS_INCLUDE_DIRS /usr/include/hiredis)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${HIREDIS_INCLUDE_DIRS}
)

add_executable(control_demo src/testcases/control_demo.cpp)
add_executable(control_kdl_demo src/testcases/control_kdl.cpp)
add_executable(control_akdl_demo src/testcases/control_akdl.cpp)
add_executable(control_adb_demo src/testcases/control_db_basic.cpp)
add_executable(urdf_test_demo src/urdf_test.cpp)
add_executable(arobot_walk src/arobot_walk.cpp)
add_executable(arobot_walk_arm src/arobot_walk_arm.cpp)
add_executable(arobot_walk_cmd src/arobot_walk_cmd.cpp)
add_executable(arobot_breakwalk_cmd src/arobot_breakwalk_cmd.cpp)
add_executable(trajectory_send_node src/trajectory_send_node.cpp)

target_link_libraries(control_demo ${catkin_LIBRARIES})
target_link_libraries(control_kdl_demo ${catkin_LIBRARIES})
target_link_libraries(control_akdl_demo ${catkin_LIBRARIES})
target_link_libraries(control_adb_demo ${catkin_LIBRARIES})
target_link_libraries(urdf_test_demo ${catkin_LIBRARIES})
target_link_libraries(arobot_walk ${catkin_LIBRARIES})
target_link_libraries(arobot_walk_arm ${catkin_LIBRARIES})
target_link_libraries(arobot_walk_cmd ${catkin_LIBRARIES})
target_link_libraries(arobot_breakwalk_cmd ${catkin_LIBRARIES})
target_link_libraries(trajectory_send_node ${catkin_LIBRARIES})
