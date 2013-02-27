/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010. David Feil-Seifer
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>

// For moving the robot on the map
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "p2os_publisher" );
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  ros::Rate loop_rate(10);
  
  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);
  
  // Odometry test
  bool publish_odom = true;
  n_.param("publish_odom_test", publish_odom, publish_odom);
  ros::Publisher odom_pub;
  if(publish_odom) odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // Sonar Range Test
  bool publish_sonar = true;
  n_.param("publish_sonar_test", publish_sonar, publish_sonar);
  ros::Publisher sonar_pub;
  if(publish_sonar) sonar_pub = n.advertise<sensor_msgs::Range>("Sonar", 1000);;
  sensor_msgs::Range sonar;
  sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar.field_of_view = (10.0/180.0) * 3.14;
  sonar.min_range = 0.0;
  sonar.max_range = 10.0;
  
  
  double wheel_rot = 0.0;
  
  double x  = 0.0;
  double y  = 0.0;
  double th = 0.0;

  double vx  = 0.2;
  double vy  = 0.0; 
  double vth =  vx; // Circle 
  double vw  = 1.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  while( n.ok() )
	{
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
		
		// Sonar Range Test
		if(publish_sonar)
		{
      for(int i=1;i<=16;i++)
      {
        sonar.header.stamp = ros::Time::now(); 
        sonar.range        = 1.3;
        
        char buff[64];
        sprintf(buff, "Sonar_f%d", i);
        sonar.header.frame_id = buff; 
	      sonar_pub.publish(sonar);
	      
		    ros::spinOnce();
		    loop_rate.sleep();
		  }
		}
		    
		// Wheel joints
    sensor_msgs::JointState js;
	  wheel_rot += (vx / (2 * 3.14 * 0.18)) * dt;
	  
    // P3-DX
	  js.name.push_back(std::string("base_swivel_joint"));
	  js.position.push_back(0);
	  js.name.push_back(std::string("swivel_hubcap_joint"));
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("base_left_hubcap_joint"));
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("base_right_hubcap_joint"));
	  js.position.push_back(wheel_rot);
	
	  // P3-AT
	  js.name.push_back(std::string("base_front_left_hub_joint"));
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("base_front_right_hub_joint"));
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("p3at_back_left_hub_joint"));
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("p3at_back_right_hub_joint"));
	  js.position.push_back(wheel_rot);

    js.header.frame_id="base_link";
    js.header.stamp = ros::Time::now();
		joint_state_publisher.publish(js);
		
		
    // Odometry given the velocities of the robot
    if(publish_odom)
    {
      double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
      double delta_th = vth * dt;

      x  += delta_x;
      y  += delta_y;
      th += delta_th;

      // since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      // first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      // send the transform
      odom_broadcaster.sendTransform(odom_trans);

      // next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      // set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      // set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      // publish the message
      odom_pub.publish(odom);
    }
		
		ros::spinOnce();
		loop_rate.sleep();
	  last_time = current_time;
	}
}
