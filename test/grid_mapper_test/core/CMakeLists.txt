project(grid_mapper)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME})

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SRC ${SRC}
)

add_executable(${CMAKE_PROJECT_NAME}_node main.cc ${SRC})
target_link_libraries(${CMAKE_PROJECT_NAME}_node ${Eigen3_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})