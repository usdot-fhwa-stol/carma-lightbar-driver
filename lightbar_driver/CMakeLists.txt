cmake_minimum_required(VERSION 2.8.3)
project(lightbar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

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
#  LIBRARIES lightbar_driver
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

file(GLOB_RECURSE headers */*.hpp */*.h)

## Declare a C++ executable

add_executable(${PROJECT_NAME}_node
                ${headers}
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
install(DIRECTORY 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        PATTERN ".svn" EXCLUDE)

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

