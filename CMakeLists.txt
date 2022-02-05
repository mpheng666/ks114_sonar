cmake_minimum_required(VERSION 3.0.2)
project(ks114_sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  #message_generation
)

include_directories(
  include(${catkin_INCLUDE_DIRS})
  
)

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS roscpp 
  DEPENDS
)

find_package(PkgConfig REQUIRED)

pkg_check_modules (SERIAL REQUIRED serial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

set(SOURCES src/ks114_main.cpp src/ks114.cpp)
set(SOURCES2 src/ks114_utility.cpp)

add_executable(ks114_sonar ${SOURCES})
add_executable(ks114_utility ${SOURCES2})

target_link_libraries(ks114_sonar
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
   ${SERIAL_LIBRARIES}
)
target_link_libraries(ks114_utility
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
   ${SERIAL_LIBRARIES}
)
