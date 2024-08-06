project(${CMAKE_PROJECT_NAME}_ros2)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_ros2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2 -pthread -pg")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(Eigen3 REQUIRED)
SET(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

set(SRC ${SRC})

add_executable(grid_odometer_node grid_odometer_node.cc ${SRC})
target_link_libraries(grid_odometer_node grid_odometer_core ${Eigen3_LIBRARIES} ) # grid_odometer
ament_target_dependencies(grid_odometer_node rclcpp sensor_msgs nav_msgs)
install(TARGETS grid_odometer_node
  DESTINATION lib/${CMAKE_PROJECT_NAME}) # install executable
