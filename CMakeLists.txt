cmake_minimum_required(VERSION 3.0.2)
project(ks114_sonar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set(PROJECT_VERSION_MAJOR 0)
# set(PROJECT_VERSION_MINOR 1)
# set(PROJECT_VERSION_PATCH 0)
# set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
  
# dynamic reconfig
# find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/sonar.cfg
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
  
  pkg_check_modules(SERIAL REQUIRED serial)
  include_directories(${SERIAL_INCLUDE_DIRS})
  link_directories(${SERIAL_LIBRARY_DIRS})
  
  # ks114 sonar base 
  set(SOURCES1 src/ks114_sonar_node.cpp src/ks114_sonar.cpp)
  add_executable(ks114_sonar_node ${SOURCES1})
  target_link_libraries(ks114_sonar_node
  ${SERIAL_LIBRARIES}
  )
  
  # ks114 sonar lib
  add_library(ks114_sonar ${SOURCES1})
  target_link_libraries(ks114_sonar ${SERIAL_LIBRARIES})
  
  
  # ks114 sonar utility
  set(SOURCES2 src/ks114_sonar_utility.cpp src/ks114_utility_node.cpp)
  add_executable(ks114_sonar_utility ${SOURCES2})
  target_link_libraries(ks114_sonar_utility
  ks114_sonar
  )
  
  # sensors manager
  set(SOURCES3 src/sensors_manager_node.cpp src/sensors_manager.cpp)
  add_executable(sensors_manager_node ${SOURCES3})
  add_dependencies(sensors_manager_node ${PROJECT_NAME}_gencfg)
  target_link_libraries(sensors_manager_node
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ks114_sonar
  )

  # # comms_handler_library
  # add_library(comms_handler_)

  # comms handler
  add_executable(comms_handler_test_node test/comms_handler_test_node.cpp src/comms_handler.cpp)
  target_link_libraries(comms_handler_test_node
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

  # sensor_reader_node
  add_executable(sensor_reader_node src/sensor_reader_node.cpp src/sensor_reader.cpp src/comms_handler.cpp src/ks114_sonar.cpp)
  target_link_libraries(sensor_reader_node
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SERIAL_LIBRARIES}
  )
  
  