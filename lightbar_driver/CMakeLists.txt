
# Copyright (C) 2022 LEIDOS.
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

cmake_minimum_required(VERSION 3.5)
project(lightbar_driver)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(CURL REQUIRED)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)


find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(worker_lib lightbar_driver_controller_lib)
set(node_exec lightbar_driver_exec)
set(node_lib lightbar_driver_application_node)

## Specify additional locations of header files
include_directories(
 include
  ${CURL_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

# Build
ament_auto_add_library(${worker_lib}
        src/lightbar_driver_controller.cpp
)
ament_auto_add_library(${node_lib} SHARED
        src/lightbar_driver_application.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "lightbar_driver::LightBarApplication")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_lib}
        ${worker_lib}
        ${CURL_LIBRARIES} 
        ${PUGIXML_LIBRARIES} 
)

target_link_libraries(${node_exec}
        ${node_lib}
        ${CURL_LIBRARIES} 
        ${PUGIXML_LIBRARIES} 
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_lightbar_driver_controller test/test_lightbar_driver_controller.cpp)

  ament_target_dependencies(test_lightbar_driver_controller ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_lightbar_driver_controller ${node_lib})
endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE launch
)

