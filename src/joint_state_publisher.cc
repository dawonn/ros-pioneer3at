/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010. David Feil-Seifer
 *  Copyright (c) 2013. Dereck Wonnacott
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

double tf_future_date;

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "urdf_joint_state_publisher" );
  ros::NodeHandle n;
  ros::NodeHandle n_("~");  
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  n_.param<double>("tf_future_date", tf_future_date,  0.1);
  
  double wheel_vel = 0.0;
  double wheel_rot = 0.0;
  double vx  = 0.2;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate loop_rate(10);
  
  while( n.ok() )
	{
		current_time = ros::Time::now(); 
		double dt = (current_time - last_time).toSec();
				    
		// Wheel joints
    sensor_msgs::JointState js;
	  wheel_vel  = (vx / (2 * 3.14 * 0.18));
	  wheel_rot += wheel_vel * dt;
	                                            
	  js.name.push_back(std::string("front_left_axle_joint"));
	  js.velocity.push_back(wheel_vel);
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("front_right_axle_joint"));
	  js.velocity.push_back(wheel_vel);
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("rear_left_axle_joint"));
	  js.velocity.push_back(wheel_vel);
	  js.position.push_back(wheel_rot);
	  js.name.push_back(std::string("rear_right_axle_joint"));
	  js.velocity.push_back(wheel_vel);
	  js.position.push_back(wheel_rot);

    ros::Duration future_date(tf_future_date);
    js.header.stamp = ros::Time::now() + future_date;
		joint_state_pub.publish(js);
		
		ros::spinOnce();
		loop_rate.sleep();
	  last_time = current_time;
	}
}
