cmake_minimum_required(VERSION 3.10)
project(AVATAR)

#add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CXX_STANDARD 11)
#add_compile_options(-fPIC)

find_package(catkin REQUIRED COMPONENTS 
	cv_bridge
	geometry_msgs 
	image_transport 
	nodelet 
	roscpp 
	sensor_msgs 
	std_msgs 
	std_srvs
	tf
	tf_conversions
	eigen_conversions
	)



find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 	 
								geometry_msgs  
								tf 
								tf_conversions
								sensor_msgs 
								std_msgs
								eigen_conversions
               INCLUDE_DIRS include
               LIBRARIES 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(ImgProcessing src/ImageProcessing.cpp)
target_link_libraries(ImgProcessing ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

