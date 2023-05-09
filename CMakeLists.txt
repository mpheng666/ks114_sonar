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
set(SOURCES1 src/ks114_sonar_node.cpp src/ks114_sonar.cpp)
add_executable(ks114_sonar_node ${SOURCES1})
target_link_libraries(ks114_sonar_node
${SERIAL_LIBRARIES}
)

# comms_handler
add_executable(comms_handler_test_node test/comms_handler_test_node.cpp src/comms_handler.cpp)
target_link_libraries(comms_handler_test_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
${SERIAL_LIBRARIES}
${Boost_LIBRARIES}
)

# sonar_reader_node
add_executable(sonar_reader_node src/sonar_reader_node.cpp src/sonar_reader.cpp src/comms_handler.cpp src/ks114_sonar.cpp)
target_link_libraries(sonar_reader_node
${roscpp_LIBRARIES}
${catkin_LIBRARIES}
${SERIAL_LIBRARIES}
)

# ks114 sonar utility
# set(SOURCES2 src/ks114_sonar_utility.cpp src/ks114_utility_node.cpp)
# add_executable(ks114_sonar_utility ${SOURCES2})
# target_link_libraries(ks114_sonar_utility
# ks114_sonar
# )

# sensors manager
# set(SOURCES3 src/sensors_manager_node.cpp src/sensors_manager.cpp)
# add_executable(sensors_manager_node ${SOURCES3})
# target_link_libraries(sensors_manager_node
# ${roscpp_LIBRARIES}
# ${catkin_LIBRARIES}
# ks114_sonar
# )


