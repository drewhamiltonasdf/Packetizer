#
# CMakeLists for drop-in compatibility with ROS
# and a Teensy (I used 3.6)
#
# This is a subtle adjustment to hideataki's code, mostly adding CMakeLists
# so we can parse packets coming in from the Teensy
# using the same library that we use to make the
# packets in the first place.
#
# by Drew Hamilton (contact@drewhamilton.net)
#

cmake_minimum_required(VERSION 3.0.2)
project(ros_encoding)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    packetizer_crc_hw
)

catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
  LIBRARIES ${PROJECT_NAME}  
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

#set_target_properties(packetizer_crc_hw PROPERTIES LINKER_LANGUAGE CXX)

## Declare a C++ library
add_library(${PROJECT_NAME}
   ros_encoding.cpp
)

target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )