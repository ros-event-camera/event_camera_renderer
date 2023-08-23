#
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_compile_options(-Wall -Wextra -pedantic -Werror)
add_definitions(-DUSING_ROS_1)

find_package(catkin REQUIRED COMPONENTS
  event_camera_msgs
  event_camera_codecs
  nodelet
  sensor_msgs
  roscpp
  image_transport)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS})

#
# --------- renderer library
#
add_library(renderer
  src/renderer_ros1.cpp
  src/display.cpp
  src/time_slice_display.cpp
  src/sharp_display.cpp)

target_link_libraries(renderer ${catkin_LIBRARIES})

#
# --------- nodelet
#
add_library(renderer_nodelet src/renderer_nodelet.cpp)
target_link_libraries(renderer_nodelet ${catkin_LIBRARIES})

#
# -------- node
#
add_executable(renderer_node src/renderer_node_ros1.cpp)
target_link_libraries(renderer_node renderer ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS renderer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS renderer renderer_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch")

#############
## Testing ##
#############

# To be done...
