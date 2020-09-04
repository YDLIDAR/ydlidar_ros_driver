# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 2.8)

project(ydlidar_ros_driver)


#list(APPEND CMAKE_MODULE_PATH "/home/tony/gitlab/build/install/share/YDLIDAR_SDK")
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)


#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS})

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  	message_generation
	roscpp
	rospy
	sensor_msgs
	)

##################LaserFan ##########################
add_message_files(DIRECTORY msg
  FILES
  LaserFan.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
	roscpp rospy sensor_msgs message_runtime
	)
#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#Include directories
include_directories(${catkin_INCLUDE_DIRS} ${YDLIDAR_SDK_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

#Include directories
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/laser_fuse
)


set(LASER_FUSE_PATH "./laser_fuse")

FILE(GLOB LASER_FUSE_SRC 
  "${LASER_FUSE_PATH}/*.cpp"
)



#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/ydlidar_ros_driver.cpp NoiseFilter/NoiseFilter.cpp NoiseFilter/NoiseFilter.h ${LASER_FUSE_SRC})

#---------------------------------------------------------------------------------------
# add projects that depend on
#---------------------------------------------------------------------------------------
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${YDLIDAR_SDK_LIBRARIES}
	)


#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

