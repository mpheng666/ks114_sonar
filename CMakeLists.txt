cmake_minimum_required(VERSION 3.0.2)
project(ks114_sonar)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
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
  
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL REQUIRED serial)
include_directories(${SERIAL_INCLUDE_DIRS})
link_directories(${SERIAL_LIBRARY_DIRS})

# ks114_sonar 
set(SOURCES1 node/ks114_sonar_node.cpp src/ks114_sonar.cpp)
add_executable(ks114_sonar_node ${SOURCES1})
target_link_libraries(ks114_sonar_node
${SERIAL_LIBRARIES}
)

# sonar_reader_node
add_executable(sonar_reader_node node/sonar_reader_node.cpp src/sonar_reader.cpp src/comms_handler.cpp src/ks114_sonar.cpp)
target_link_libraries(sonar_reader_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
${SERIAL_LIBRARIES}
)

# sonars_manager_node
add_executable(sonars_manager_node node/sonars_manager_node.cpp src/sonars_manager.cpp src/sonar_reader.cpp src/comms_handler.cpp src/ks114_sonar.cpp)
target_link_libraries(sonars_manager_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
${SERIAL_LIBRARIES}
)

# ks114 sonar utility_node
add_executable(ks114_sonar_utility_node node/ks114_utility_node.cpp src/ks114_sonar_utility.cpp src/sonar_reader.cpp src/comms_handler.cpp src/ks114_sonar.cpp)
target_link_libraries(ks114_sonar_utility_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
${SERIAL_LIBRARIES}
)


