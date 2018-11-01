# ros-pioneer3at
Programs that show how to drive a Pioneer3-AT robot with ros (I used it for Gazebo).

## This fork
I donwloaded this example to learn how to use ROS (system for robot communication) and Gazebo (robot simulator) in tandem. This project contains more content than what I used, but at least for my case I noticed 2 shortcomings: there's no README in the original, and it doesn't work with the latest version of Gazebo, which in my case is version 9.

## README
Instead of a README.md the project has a [google doc](https://docs.google.com/document/d/1-HmQuTe955WDy5t9Q70rw00o4WJjFePuAhqxbgarA1Q/edit), which I'll try to paste below.

### Introduction
> This is a quick-start guide to using a Pioneer3AT Mobile Robot with ROS. It should allow you to drive a robot via a GUI application or a PS3 controller. If you have a SICK or Hokuyo LMS device, you can also make a 2D map of an environment and be able to give the robot Autonomous navigation commands from a GUI application. 

> If you have any problems, get stuck, lost, confused, find errors, just want to say thanks, etc, highlight the area of the document and leave a comment. I’ll try to keep this up to date as best as I can whenever I receive feedback. :)

### System Installation
#### ROS Installation
> Install wiki page works well: http://www.ros.org/wiki/hydro/Installation/Ubuntu
> Select the -desktop-full package.

#### Gazebo Standalone
> __THIS FORK FIXES HAVING TO USE GAZEBO 1.6. SO FEEL FREE TO DOWNLOAD A CURRENT VERSION.__ 

> _The Pioneer3AT ROS package requires some features in Gazebo-1.6 which is still under development, so we need to install from source at this time._
 `mkdir ~/gazebo`
 `cd ~/gazebo`
  `hg clone https://bitbucket.org/osrf/gazebo gazebo`
  `mkdir -p ~/gazebo/gazebo/build`
  `cd ~/gazebo/gazebo/build`
  `cmake ..`
  `make -j 8         # Replace 8 with the number of cores in your system`
  `sudo make install`

#### Workspace Setup:
> Check the ROS website for installing and setting up the workspace

#### Userspace Installation 
##### RosAria
  > `cd ~/catkin_ws/src`<br>
  > `git clone https://github.com/amor-ros-pkg/rosaria.git`<br>
  > `cd ..`<br>
  > `rosdep install rosaria`<br>
  > `catkin_make`<br>
  > `source ~/.bashrc`<br>
##### Pioneer 3AT
  > `cd ~/catkin_ws/src`<br>
  > `git clone git://github.com/dawonn/ros-pioneer3at.git`<br>
  > `cd ..`<br>
  > `rosdep install pioneer3at`<br>
  > `catkin_make`<br>
  > `source ~/.bashrc

##### Gazebo
  > `echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc`<br>
  > `echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc` _This line may depend on how you install it_<br>
  > `source ~/.bashrc

##### Gazebo Models
  > `gazebo`

  > 1) Click the insert tab on the top left of the screen<br>
  > 2) Expand the ‘Model Database’<br>
  > 3) Insert a Pioneer 3AT model into the world<br>
  > 4) Insert a Willow Garage model into the world<br>
  > 5) Close Gazebo

  > Note: This process downloads a copy of the models to ~/.gazebo/models/

#### Using Gazebo Simulator
##### Basic Configuration
  > `gedit ~/catkin_ws/src/ros-pioneer3at/launch/hardware.launch`<br>
  > 1) Comment the physical robot and lidar drivers<br>
  > 2) Uncomment the gazebo robot and lidar drivers<br>
  > Example:<br>
  > `<!-- Select the robot-driver you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/core/gazebo.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/core/rosaria.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/core/p2os_driver.launch" /> -->`<br>

  > `<!-- Select the laser you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/lidar/gazebo_hokuyo.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/lidar/sicklms.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/lidar/hokuyo.launch" /> -->`<br>

  > `gedit ~/catkin_ws/src/ros-pioneer3at/launch/ui.launch`<br>	
  > 1) Select the control method you want to use<br>
  > `<!-- Select the control method you wish to use -->`<br>
  > `<include file="$(find pioneer3at)/launch/control/rqt_robot_steering.launch" />`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/control/ps3joy.launch" /> -->`<br>
  > `<!-- <include file="$(find pioneer3at)/launch/control/xboxjoy.launch" /> -->`<br>

  > `sudo gedit ~/.gazebo/models/pioneer3at/model.sdf`<br>
  > 1) Uncomment the hokuyo and skidsteerdrive plugin near the bottom<br>
  > 2) Remove the <MaxForce> tag<br>
  > Example:<br>
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

  > `<plugin name="SkidSteerDrivePlugin" filename="libSkidSteerDrivePlugin.so" />`<br>

##### Launch Control Demo __(According to what I did)
  > `cd catkin_ws/src/pioneer3at/launch/core`<br>
  > `roslaunch gazebo.launch`<br>
  
  > Then, in another terminal:
  > `gzclient`

  With this most basic example you can send movement messages to the robot along the `/Pioneer3AT/cmd_vel` topic.

## Updating basic usage with Gazebo for version 9
### Link ROS packages
### Switch to Ignition libraries
