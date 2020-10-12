cmake_minimum_required(VERSION 2.4.6)
project(msdi_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
  message_generation
  image_transport
)

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)

add_message_files(
  FILES
  Flag.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
    message_filters
    message_runtime
    image_transport
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(msdi_rld_data src/rgbLWIRDepth.cpp)
target_link_libraries(msdi_rld_data
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})


