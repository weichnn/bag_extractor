project(cppUtilsRGBD)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++11)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

# set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev )
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(" OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS}")

add_library(Utils SHARED
Utils.cpp
)

target_link_libraries(Utils
${OpenCV_LIBS}
)


add_executable (projDepth projDepth.cpp)
target_link_libraries (projDepth Utils)


add_executable (undistort undistort.cpp)
target_link_libraries (undistort Utils)

