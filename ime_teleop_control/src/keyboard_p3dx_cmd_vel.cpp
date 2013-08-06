/*
 * teleop_keyboard_control.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: barcelosandre
 */

//#include "sstream"
//#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <signal.h>
double max_speed = 0.100; // m/second
double max_turn = 60.0*M_PI/180.0; // rad/second

  #define KEYCODE_A 0x61
  #define KEYCODE_D 0x64
  #define KEYCODE_S 0x73
  #define KEYCODE_W 0x77

  class TeleopRoboCVKeyboard
  {
    private:
    geometry_msgs::Twist cmd;

    ros::NodeHandle n_;
    ros::Publisher drive_pub_;

    public:
    void init()
    {
      cmd.linear.x = 0;
      cmd.angular.z = 0;

      drive_pub_ = n_.advertise<geometry_msgs::Twist>("/pioneer3dx_gazebo/cmd_vel", 100);

      ros::NodeHandle n_private("~");
    }

    ~TeleopRoboCVKeyboard()   { }
    void keyboardLoop();
  };

  int kfd = 0;
  struct termios cooked, raw;

  void quit(int sig)
  {
    tcsetattr(kfd, TCSANOW, &cooked);
   exit(0);
  }

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "drive_cmd_publisher");

    TeleopRoboCVKeyboard tpk;
    tpk.init();

    signal(SIGINT,quit);

    tpk.keyboardLoop();

    return(0);
  }

  void TeleopRoboCVKeyboard::keyboardLoop()
  {
    char c;
   bool dirty=false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'WS' to forward/back");
    puts("Use 'AD' to left/right");

    for(;;)
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      switch(c)
      {
        // Walking
      case KEYCODE_W:
    puts("throttle");
    cmd.linear.x = 1;
        dirty = true;
        break;
      case KEYCODE_S:
    puts("brake");
    cmd.linear.x = 0;
        dirty = true;
        break;
      case KEYCODE_A:
    puts("turn left");
    cmd.angular.z = cmd.angular.z - 0.1;
        dirty = true;
        break;
      case KEYCODE_D:
    puts("turn right");
    cmd.angular.z = cmd.angular.z + 0.1;
        dirty = true;
        break;
      }


      if (dirty == true)
      {
        drive_pub_.publish(cmd);
      }

    }
  }

