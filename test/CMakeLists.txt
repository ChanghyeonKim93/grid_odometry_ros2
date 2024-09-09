project(${CMAKE_PROJECT_NAME}_test)
message("   -- [PROJECT]: " ${CMAKE_PROJECT_NAME}_test)

set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})
# include_directories ("/usr/include/opencv4/")
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SRC ${SRC}
)

add_executable(${CMAKE_PROJECT_NAME}_test test.cc)
target_link_libraries(${CMAKE_PROJECT_NAME}_test ${Eigen3_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
install(TARGETS ${CMAKE_PROJECT_NAME}_test 
  DESTINATION /usr/lib/${CMAKE_PROJECT_NAME}
  CONFIGURATIONS Release)

add_executable(${CMAKE_PROJECT_NAME}_test_monte_carlo_localization_1d test_monte_carlo_localization_1d.cc)
target_link_libraries(${CMAKE_PROJECT_NAME}_test_monte_carlo_localization_1d ${Eigen3_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS ${CMAKE_PROJECT_NAME}_test_monte_carlo_localization_1d 
  DESTINATION /usr/lib/${CMAKE_PROJECT_NAME}
  CONFIGURATIONS Release)

message("DIRECTORY FOR INSTALL :" /usr/lib/${CMAKE_PROJECT_NAME})