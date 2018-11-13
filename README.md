# ros-pioneer3at
Programs that show how to drive a Pioneer3-AT robot with ros.

## README
This README is adapted from this [google doc](https://docs.google.com/document/d/1C_GdAAQck-IT4H5GnsH3j6OezUbehUphtlnn8XU4XBk/edit), which has some better formatting and diagrams if the instructions here are confusing.

## Introduction
> This is a quick-start guide to using a Pioneer3AT Mobile Robot with ROS. It should allow you to drive a robot via a GUI application or a PS3 controller. If you have a SICK or Hokuyo LMS device, you can also make a 2D map of an environment and be able to give the robot Autonomous navigation commands from a GUI application. 

> If you have any problems, get stuck, lost, confused, find errors, just want to say thanks, etc, highlight the area of the document and leave a comment. I’ll try to keep this up to date as best as I can whenever I receive feedback. :)

## System Installation
### ROS Installation
> Install wiki page works well: http://www.ros.org/wiki/melodic/Installation/Ubuntu
> Select the -desktop-full package.
https://docs.google.com/document/d/1-HmQuTe955WDy5t9Q70rw00o4WJjFePuAhqxbgarA1Q/edit
### Workspace Setup:
> Check the ROS website for installing and setting up the workspace

### Gazebo Dependencies
> Apparently, Gazebo 9 a new set of dependencies from Ignition was introduced. Follow [these instructions](http://gazebosim.org/tutorials?tut=install_dependencies_from_source) to download them.

### Userspace Installation 
#### RosAria
  > `cd ~/catkin_ws/src`<br>
  > `git clone https://github.com/amor-ros-pkg/rosaria.git`<br>
  > `cd ..`<br>
  > `rosdep install rosaria`<br>
  > `catkin_make`<br>
  > `source ~/.bashrc`<br>
#### Pioneer 3AT
  > `cd ~/catkin_ws/src`<br>
  > `git clone git://github.com/dawonn/ros-pioneer3at.git`<br>
  > `cd ..`<br>
  > `rosdep install pioneer3at`<br>
  > `catkin_make`<br>
  > `source ~/.bashrc`

#### Gazebo
_For directions that required Gazebo 1.6, see the [google doc](https://docs.google.com/document/d/1C_GdAAQck-IT4H5GnsH3j6OezUbehUphtlnn8XU4XBk/edit)._
  > Follow the instructions at [gazebosim.org](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) for download and installation.<br>
  > `echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc`<br>
  > `echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc` This line may depend on how you install it.<br>
  > `source ~/.bashrc`

#### Gazebo Models
  > `gazebo`

  > 1) Click the insert tab on the top left of the screen<br>
  > 2) Expand the ‘Model Database’<br>
  > 3) Insert a Pioneer 3AT model into the world<br>
  > 4) Insert a Willow Garage model into the world<br>
  > 5) Close Gazebo

  > Note: This process downloads a copy of the models to ~/.gazebo/models/

### Using Gazebo Simulator
#### Basic Configuration
  > `gedit ~/catkin_ws/src/ros-pioneer3at/launch/hardware.launch` (gedit can be replaced with any text editor)<br>
  > 1) Comment the physical robot and lidar drivers<br>
  > 2) Uncomment the gazebo robot and lidar drivers<br>
  
  > `<!-- Select the robot-driver you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/core/gazebo.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/core/rosaria.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/core/p2os_driver.launch" /> -->`<br>

  > `<!-- Select the laser you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/lidar/gazebo_hokuyo.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/lidar/sicklms.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/lidar/hokuyo.launch" /> -->`<br>

  > `gedit ~/catkin_ws/src/ros-pioneer3at/launch/ui.launch`<br>	
  > 1) Select the control method you want to use.<br>
  > `<!-- Select the control method you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/control/rqt_robot_steering.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/control/ps3joy.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/control/xboxjoy.launch" /> -->`<br>

  > `sudo gedit ~/.gazebo/models/pioneer3at/model.sdf`<br>
  > 1) Uncomment the skidsteerdrive plugin near the bottom:<br>
  > 2) Remove the <MaxForce> tag<br>
  > 3) Add the following:<br>
  > `<include>`<br>
  > `  <uri>model://hokuyo</uri>`<br>
  > `  <pose>0.2 0 0.13 0 0 0</pose>`<br>
  > `</include>`<br>
  > `<joint name="hokuyo_joint" type="revolute">`<br>
  > `  <child>hokuyo::link</child>`<br>
  > `  <parent>chassis</parent>`<br>
  > `  <axis>`<br>
  > `    <xyz>0 0 1</xyz>`<br>
  > `    <limit>`<br>
  > `      <upper>0</upper>`<br>
  > `      <lower>0</lower>`<br>
  > `    </limit>`<br>
  > `  </axis>`<br>
  > `</joint>`

#### Launch Control Demo
This demo is the most basic example; you can send movement messages to the robot along the `/Pioneer3AT/cmd_vel` topic.

  > `cd catkin_ws/src/pioneer3at/launch/`<br>
  > `roslaunch hardware.launch`<br>
  
  > Then, in another terminal:
  > `gzclient`

#### Control Demo Explanation
File paths here are all prefixed by `~catkin_ws/src/pioneer3at`.

`launch/hardware.launch` runs `launch/core/gazebo.launch`.

`gazebo.launch` in turn opens a world (`config/gazebo/wg_world.sdf`) with all the models needed in a gazebo server, and runs a ROS node from `src/gazebo_bridge.cc` that connects Gazebo to ROS communication so the robot can be controlled.

`hardware.launch` then enables the laser attached to the robot model with `launch/lidar/gazebo_hokuyo.launch`.

`gazebo_hokuyo.launch` runs a laser-scanning ROS node from `src/gazebo_laserscan.cc` which publishes information from the laser sensor.

Finally, `hardware.launch` runs `launch/core/urdf.launch` and `cmd_vel_mux.launch`. The first opens a ROS node that publishes location+orientation information about the robot, and the second seems to remap a ROS topic so robot-control commands can be communicated along Pioneer3AT/cmd\_vel. _(This information should be reviewed)._
  
__If you want to run more complex examples, check out `demo_navigation_amcl.launch` and `demo_navigation_gmapping.launch`.__
