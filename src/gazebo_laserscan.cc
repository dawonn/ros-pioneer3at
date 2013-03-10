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

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

std::string NodeName;
ros::Publisher ros_laserscan_pub;

/////////////////////////////////////////////////
void gz_laserscan_Callback(ConstLaserScanStampedPtr &msg_in)
{ 
  //std::cout << ">> " << msg_in->DebugString() << std::endl;
  sensor_msgs::LaserScan msg_out;
  msg_out.angle_min         = msg_in->scan().angle_min();
  msg_out.angle_max         = msg_in->scan().angle_min();
  msg_out.angle_increment   = msg_in->scan().angle_step();
  msg_out.time_increment    = 0;
  msg_out.scan_time         = 0;
  msg_out.range_min         = msg_in->scan().range_min();
  msg_out.range_max         = msg_in->scan().range_max();
  uint32_t sec = msg_in->time().sec();
  uint32_t nsec = msg_in->time().nsec();
  msg_out.header.stamp      = ros::Time( sec, nsec );
  msg_out.header.frame_id   = NodeName + "/laser_scan";
  
  for(int i = 0; i < msg_in->scan().ranges_size(); i++){
    msg_out.ranges.push_back( msg_in->scan().ranges(i) );
  }
  for(int i = 0; i < msg_in->scan().intensities_size(); i++){
    msg_out.intensities.push_back( msg_in->scan().intensities(i) );
  }

  ros_laserscan_pub.publish(msg_out);
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS
  ros::init(argc, argv, "Pioneer3AT");
  NodeName = ros::this_node::getName();
  ros::NodeHandle n_("~");
  ros_laserscan_pub = n_.advertise<sensor_msgs::LaserScan>("laserscan", 10);
  
  
  // Initialize Gazebo
  gazebo::transport::init();
  gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
  gz_node->Init();
  gazebo::transport::run();
  gazebo::transport::SubscriberPtr gz_lasersacn_sub = gz_node->Subscribe( std::string("~") +
   NodeName + std::string("/hokuyo/link/laser/scan"), gz_laserscan_Callback);
  
  // Spin
  ros::spin();

  // Shutdown
  gazebo::transport::fini();
}




