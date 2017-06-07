cmake_minimum_required(VERSION 2.8.3)
project(arobot_hw)


option(ecat "This is a default option for ethercat compile" ON)
message(STATUS "build with ethercat switch:" ${ecat})

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs 
			 arobot_db arobot_kdl 
			 controller_manager control_toolbox controller_interface transmission_interface)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

if(${ecat} STREQUAL "ON")

add_definitions("-DECAT")
## ethercat and artrobot_ethercat
set(ETHERCAT_INCLUDE_DIR /usr/local/include)
set(ETHERCAT_LIBRARY /usr/local/lib/libethercat.so.1.0.0)

set(ARTROBOT_ETHERCAT_INCLUDE_DIR /usr/local/include/artrobot_ethercat)
set(ARTROBOT_ETHERCAT_LIBRARY /usr/local/lib/libartrobot_ethercat.so)

endif()
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DriverDetection.srv 
   SelfDetection.srv 
   SensorDetection.srv
   SensorCalibration.srv
   ActuatorDetection.srv
   ActuatorControl.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp arobot_db arobot_kdl controller_manager control_toolbox controller_interface transmission_interface
				   message_runtime std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(HIREDIS_INCLUDE_DIRS /usr/include/hiredis)

if(${ecat} STREQUAL "ON")
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${ETHERCAT_INCLUDE_DIR}
        ${ARTROBOT_ETHERCAT_INCLUDE_DIR}
        ${HIREDIS_INCLUDE_DIRS}
)
else()
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${HIREDIS_INCLUDE_DIRS}
)
endif()

## Declare a C++ library


if(${ecat} STREQUAL "ON")
 add_library(${PROJECT_NAME}
   src/arobot_hw.cpp
   src/arobot_trans.cpp
         src/controller_main.cpp
         src/AngleToNumber.cpp
         include/AngleToNumber.h
         include/ecat_init.h
         src/ecat_init.cpp
)

else()
 add_library(${PROJECT_NAME}
   src/arobot_hw.cpp
   src/arobot_trans.cpp
   src/controller_main.cpp
)
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(control_decision_node src/control_decision_node.cpp)
 add_executable(arobot_trans_test src/arobot_trans_test.cpp)
 add_executable(controller_main src/controller_main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(control_decision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

## Specify libraries to link a library or executable target against

if(${ecat} STREQUAL "ON")
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
         ${ETHERCAT_LIBRARY}
         ${ARTROBOT_ETHERCAT_LIBRARY}
         -lpthread
 )

else()

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES})

endif()

 target_link_libraries(arobot_trans_test
	 ${catkin_LIBRARIES} ${PROJECT_NAME}
 )

target_link_libraries(controller_main
        ${catkin_LIBRARIES} ${PROJECT_NAME}
        )



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS arobot_hw
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_control_decision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
