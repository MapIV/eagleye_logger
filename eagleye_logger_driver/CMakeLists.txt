cmake_minimum_required(VERSION 2.8.3)
project(eagleye_logger_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  can_msgs
  logger_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  can_msgs
  logger_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
)

add_executable(adis16470_node src/adis16470_node.cpp  src/gnss_server.cpp)
target_link_libraries(adis16470_node ${catkin_LIBRARIES} )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

message(STATUS "*** dump start cmake variables ***")
get_cmake_property(_variableNames VARIABLES)
foreach(_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
message(STATUS "*** dump end ***")
