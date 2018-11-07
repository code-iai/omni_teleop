/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * Copyright (c) 2018, Alexis Maldonao & Georg Bartels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class TurtlebotTeleop
{
public:
  TurtlebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_x, linear_y, angular_, deadman_axis_, headctrl_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_, vel_s_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::TwistStamped last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  bool stamped_interface_selected_;
  ros::Timer timer_;

};

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_x(1),
  linear_y(0),
  angular_(2),
  deadman_axis_(4),
  headctrl_axis_(8),
  l_scale_(0.3),
  a_scale_(0.9),
  stamped_interface_selected_(true)
{
  ph_.param("axis_linear_x", linear_x, linear_x);
  ph_.param("axis_linear_y", linear_y, linear_y);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_headctrl", headctrl_axis_, headctrl_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("stamped_interface", stamped_interface_selected_, stamped_interface_selected_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;

  if (stamped_interface_selected_)
    vel_s_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
  else
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.01), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::TwistStamped vel;
  vel.header.frame_id = "/base_footprint";
  vel.header.stamp = joy->header.stamp;
  vel.twist.angular.z = a_scale_*joy->axes[angular_];
  vel.twist.linear.x = l_scale_*joy->axes[linear_x];
  vel.twist.linear.y = l_scale_*joy->axes[linear_y];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_] && (not joy->buttons[headctrl_axis_]);
}

void TurtlebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    if (stamped_interface_selected_)
      vel_s_pub_.publish(last_published_);
    else
      vel_pub_.publish(last_published_.twist);
    zero_twist_published_ = false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    last_published_.twist = geometry_msgs::Twist();
    last_published_.header.stamp = ros::Time::now();
    if (stamped_interface_selected_)
      vel_s_pub_.publish(last_published_);
    else
      vel_pub_.publish(last_published_.twist);
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  TurtlebotTeleop turtlebot_teleop;

  ros::spin();
}
