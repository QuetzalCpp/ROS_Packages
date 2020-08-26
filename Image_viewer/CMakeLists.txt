cmake_minimum_required(VERSION 2.8.3)
project(Image_viewer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Image_viewer
#  CATKIN_DEPENDS cv_bridge image_transport roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(include include/Image_viewer)

## Declare a C++ executable
add_executable(Image_viewer 
				src/Image_viewer_node.cpp 
				src/Image_viewer_nodelet.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(Image_viewer 
				${catkin_LIBRARIES}
				${OpenCV_LIBS})
