cmake_minimum_required(VERSION 2.8.3)
project(pioneer3at)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs tf)

#######################
## Gazebo Standalone ##
#######################

find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


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
add_executable(       joint_state_publisher src/joint_state_publisher.cc)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES} )

add_executable(       gazebo_bridge     src/gazebo_bridge.cc)
target_link_libraries(gazebo_bridge     ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})
                                    
add_executable(       gazebo_laserscan  src/gazebo_laserscan.cc)
target_link_libraries(gazebo_laserscan  ${catkin_LIBRARIES} 
                                        ${GAZEBO_LIBRARIES} 
                                        ${Boost_LIBRARIES} 
                                        ${PROTOBUF_LIBRARIES})

#############
## Install ##
#############
install(PROGRAMS  scripts/init_amcl
                  scripts/teleop_joy
                  scripts/gazebo_p3at
                  scripts/proto/__init__.py
                  scripts/proto/header.py
                  scripts/proto/packet.py
                  scripts/proto/pose.py
                  scripts/proto/publish.py
                  scripts/proto/quaternion.py
                  scripts/proto/subscribe.py
                  scripts/proto/time.py
                  scripts/proto/vector3d.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS joint_state_publisher
                gazebo_bridge
                gazebo_laserscan
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

