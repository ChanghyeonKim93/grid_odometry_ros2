project(multi_resolution_ndt)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIRS})

set(SRC ${SRC}
  ndt_updater.cc
)

add_executable(example_node main.cc ${SRC})
target_link_libraries(example_node 
  ${Eigen3_LIBRARIES} 
  ${OpenCV_LIBS}
)