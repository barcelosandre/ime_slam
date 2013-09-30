/*
 * joy_cmd_vel.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: barcelosandre
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>

class TeleopP3DX
{
public:
	TeleopP3DX();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopP3DX::TeleopP3DX():
  linear_(1),
  angular_(3)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/pioneer3dx/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopP3DX::joyCallback, this);
}

void TeleopP3DX::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  if(joy->axes[angular_] != 0 || joy->axes[linear_] != 0)
    {
  	  vel.angular.z = 0.7*(joy->axes[angular_]);
  	  vel.linear.x = 0.7*(joy->axes[linear_]);
  	  vel_pub_.publish(vel);
    }
    else if(joy->buttons[0] == true)
    {
      ROS_INFO("1 meter front step!");
       vel.angular.z = 0;
      //1 meter step in 2 seconds
    	vel.linear.x = 0.5;
    	vel_pub_.publish(vel);
    	sleep(2);
    	vel.linear.x = 0;
    	vel_pub_.publish(vel);
    }
    else if(joy->buttons[2] == true)
    {
      ROS_INFO("1 meter back step!");
       vel.angular.z = 0;
      //1 meter step in 2 seconds
    	vel.linear.x = -0.5;
    	vel_pub_.publish(vel);
    	sleep(2);
    	vel.linear.x = 0;
    	vel_pub_.publish(vel);
    }
    else if(joy->buttons[1] == true)
    {
		ROS_INFO("90 degrees right step!");
		vel.linear.x = 0;
		//90 degrees step in 2 seconds
		vel.angular.z = -0.7854;
		vel_pub_.publish(vel);
		sleep(2);
		vel.angular.z = 0;
		vel_pub_.publish(vel);
    }
    else if(joy->buttons[3] == true)
    {
		ROS_INFO("90 degrees left step!");
		vel.linear.x = 0;
		//90 degrees step in 2 seconds
		vel.angular.z = 0.7854;
		vel_pub_.publish(vel);
		sleep(2);
		vel.angular.z = 0;
		vel_pub_.publish(vel);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_P3DX");
  TeleopP3DX teleop_turtle;

  ros::spin();
}

