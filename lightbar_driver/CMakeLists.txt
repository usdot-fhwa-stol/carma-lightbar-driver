
# Copyright (C) 2020-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 2.8.3)
project(lightbar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cav_driver_utils
  cav_msgs
  roscpp
)


find_package(CURL REQUIRED)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)


find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lightbar_driver
  CATKIN_DEPENDS
  cav_driver_utils
  cav_msgs
  pugixml
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)



## Declare a C++ executable

add_executable(${PROJECT_NAME}_node
                src/main.cpp
                src/lightbar_driver_controller.cpp
                src/lightbar_driver_application.cpp
)
add_library(${PROJECT_NAME}_lib
                src/main.cpp
                src/lightbar_driver_controller.cpp
                src/lightbar_driver_application.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node 
                ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME}_lib
                ${catkin_EXPORTED_TARGETS}
) 

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
                ${catkin_LIBRARIES}
                ${CURL_LIBRARIES} 
                ${PUGIXML_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}_lib 
                ${catkin_LIBRARIES}
                ${CURL_LIBRARIES} 
                ${PUGIXML_LIBRARIES} 
)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
  test/test_lightbar_driver_controller.cpp
  test/test_main.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
endif()

