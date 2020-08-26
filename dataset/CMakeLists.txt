cmake_minimum_required(VERSION 2.8.3)
project(dataset)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dataset
#  CATKIN_DEPENDS cv_bridge image_transport roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${ARDRONESDK3_PATH}/lib
  ${ARSDK_LIB_DIR}
)

include_directories(include include/dataset)

## Declare a C++ executable
add_executable(dataset 
				src/dataset_node.cpp 
				src/dataset_nodelet.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dataset 
				${catkin_LIBRARIES}
				${OpenCV_LIBS})

