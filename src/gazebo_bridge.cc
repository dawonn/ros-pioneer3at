/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math/gzmath.hh>

#include <iostream>

gazebo::transport::PublisherPtr gz_vel_cmd_pub;

/////////////////////////////////////////////////
void ros_cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg_in)
{ 
  // Generate a pose
  gazebo::math::Pose pose(msg_in->linear.x,
                          msg_in->linear.y,
                          msg_in->linear.z,
                          msg_in->angular.x,
                          msg_in->angular.y,
                          msg_in->angular.z);
  
  // Convert to a pose message
  gazebo::msgs::Pose msg_out;
  gazebo::msgs::Set(&msg_out, pose);
  gz_vel_cmd_pub->Publish(msg_out);
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize Gazebo
  gazebo::transport::init();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::run();
  gz_vel_cmd_pub = node->Advertise<gazebo::msgs::Pose>("~/Pioneer3AT/vel_cmd");
  gz_vel_cmd_pub->WaitForConnection();
  
  
  // Initialize ROS
  ros::init(argc, argv, "gazebo_bridge");
  //ros::NodeHandle n;
  ros::NodeHandle n_("~");
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n_.subscribe("/Pioneer3AT/cmd_vel", 10, ros_cmd_vel_Callback);
  ros::spin();
    
  // Make sure to shut everything down.
  gazebo::transport::fini();
}
