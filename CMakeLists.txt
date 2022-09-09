cmake_minimum_required(VERSION 3.0.2)
project(ks114_sonar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

# include(CTest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  #message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp 
    std_msgs
    sensor_msgs
  DEPENDS
)

find_package(PkgConfig REQUIRED)

pkg_check_modules (SERIAL REQUIRED serial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

set(SOURCES1 src/ks114_node.cpp src/ks114.cpp)
set(SOURCES2 src/ks114_utility.cpp)

add_executable(ks114_sonar ${SOURCES1})
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
