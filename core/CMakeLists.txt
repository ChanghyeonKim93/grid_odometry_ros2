project(${CMAKE_PROJECT_NAME}_core)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_core)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SRC ${SRC}
  grid_odometer.cc
)

add_library(${CMAKE_PROJECT_NAME}_core STATIC ${SRC})
target_link_libraries(${CMAKE_PROJECT_NAME}_core ${Eigen3_LIBRARIES} ${CERES_LIBRARIES})
install(TARGETS ${CMAKE_PROJECT_NAME}_core 
  DESTINATION /usr/lib/${CMAKE_PROJECT_NAME}
  CONFIGURATIONS Release)

message("DIRECTORY FOR INSTALL :" /usr/lib/${CMAKE_PROJECT_NAME})