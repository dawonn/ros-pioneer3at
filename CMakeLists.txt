cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs tf)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p2os_urdf
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(p2os_urdf_publisher src/publisher.cc)
target_link_libraries(p2os_urdf_publisher ${catkin_LIBRARIES} )

#############
## Install ##
#############
install(PROGRAMS scripts/init_amcl
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS p2os_urdf_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  urdf 
  launch
  meshes 
  models
  config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

