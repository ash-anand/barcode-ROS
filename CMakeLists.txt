cmake_minimum_required(VERSION 2.8.3)
project(barcode)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  image_geometry
  roscpp
  rospy
  std_msgs
  image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
find_package (ZBar0)
catkin_package(
  CATKIN_DEPENDS message_runtime nodelet cv_bridge roscpp 
)
add_executable(barcode_node
	src/main.cpp
  )
target_link_libraries(barcode_node zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})

