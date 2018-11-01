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
//#include <gazebo/math/gzmath.hh> //This library was replaced with ignition::math
#include <ignition/math.hh>

#include <iostream>

gazebo::transport::PublisherPtr gz_vel_cmd_pub;
ros::Publisher ros_odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
int ros_odom_pub_seq;
std::string ros_cmd_vel_frame;
std::string ros_odom_frame;
std::string ros_child_frame;
std::string gz_model_name;
double ros_odom_tf_future_date;


/////////////////////////////////////////////////
void ros_cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg_in)
{ 
  // Generate a pose
//   gazebo::math::Pose pose(msg_in->linear.x,
//                           msg_in->linear.y,
//                           msg_in->linear.z,
//                           msg_in->angular.x,
//                           msg_in->angular.y,
//                           msg_in->angular.z);
  ignition::math::Pose3<double> pose(msg_in->linear.x,
                                     msg_in->linear.y,
                                     msg_in->linear.z,
                                     msg_in->angular.x,
                                     msg_in->angular.y,
                                     msg_in->angular.z);
  
  // Convert to a pose message
  gazebo::msgs::Pose msg_out = gazebo::msgs::Convert(pose);
//   gazebo::msgs::Set(&msg_out, pose);
  gz_vel_cmd_pub->Publish(msg_out);
}

/////////////////////////////////////////////////
void gz_odom_Callback(ConstPosesStampedPtr &msg_in)
{ 
  //std::cout << "gz_odom" << msg_in->DebugString() << std::endl;

  float x  = 0;
  float y  = 0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  
  for (int i = 0; i < msg_in->pose_size(); i++) 
    if(msg_in->pose(i).name() == gz_model_name)
    {
      //std::cout << msg_in->pose(i).DebugString() << std::endl;
      x = msg_in->pose(i).position().x();
      y = msg_in->pose(i).position().y();
      odom_quat.x = msg_in->pose(i).orientation().x();
      odom_quat.y = msg_in->pose(i).orientation().y();
      odom_quat.z = msg_in->pose(i).orientation().z();
      odom_quat.w = msg_in->pose(i).orientation().w();
      break;
    }    

  geometry_msgs::TransformStamped odom_trans;
  ros::Duration future_date(ros_odom_tf_future_date);
  odom_trans.header.stamp             = ros::Time::now() + future_date;
  odom_trans.header.frame_id          = ros_odom_frame;
  odom_trans.child_frame_id           = ros_child_frame;
  odom_trans.transform.translation.x  = x;
  odom_trans.transform.translation.y  = y;
  odom_trans.transform.translation.z  = 0.0;
  odom_trans.transform.rotation       = odom_quat;
  odom_broadcaster->sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header                 = odom_trans.header;
  odom.child_frame_id         = odom_trans.child_frame_id;
  odom.pose.pose.position.x   = x;
  odom.pose.pose.position.y   = y;
  odom.pose.pose.position.z   = 0.0;
  odom.pose.pose.orientation  = odom_quat;
  odom.twist.twist.linear.x   = 0.0;  //vx;
  odom.twist.twist.linear.y   = 0.0;  //vy;
  odom.twist.twist.angular.z  = 0.0;  //vth;
  ros_odom_pub.publish(odom);
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS
  ros::init(argc, argv, "Gazebo_Bridge");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  n_.param<std::string>("ros_odom_frame", ros_odom_frame,   "/odom");
  n_.param<std::string>("ros_cmd_vel_frame", ros_cmd_vel_frame,   "/cmd_vel");
  n_.param<std::string>("ros_child_frame", ros_child_frame, "/base_link");
  n_.param<double>("ros_odom_tf_future_date", ros_odom_tf_future_date,  0.5);
  
  n_.param<std::string>("gz_model_name", gz_model_name,   "Pioneer3AT");
  std::string gz_pose_topic;
  n_.param<std::string>("gz_pose_topic", gz_pose_topic, "~/pose/info");
  std::string gz_cmd_vel_topic;
  n_.param<std::string>("gz_cmd_vel_topic", gz_cmd_vel_topic, "~/vel_cmd");
  
  ros_odom_pub = n.advertise<nav_msgs::Odometry>(ros_odom_frame, 100);
  odom_broadcaster = new tf::TransformBroadcaster;
  ros_odom_pub_seq = 0;
    
  // When launched from a launch file we need to give Gazebo time to load
  ros::Duration(5.0).sleep();
  
  // Initialize Gazebo
  gazebo::transport::init();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::run();
  gz_vel_cmd_pub = node->Advertise<gazebo::msgs::Pose>(gz_cmd_vel_topic);
    
  // Subscribers
  gazebo::transport::SubscriberPtr gz_odom_sub = node->Subscribe(gz_pose_topic, gz_odom_Callback);
  ros::Subscriber ros_cmd_vel_sub = n.subscribe(ros_cmd_vel_frame, 1, ros_cmd_vel_Callback);
  
  // Spin
  ros::spin();

  // Shutdown
  gazebo::transport::fini();
}



