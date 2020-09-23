#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "lanros2podo.h"
#include <geometry_msgs/Point.h>
#include <stdlib.h>
#include <chrono>

rb5_ros_wrapper::MotionGoal rb5_goal;

//will send a command over the action client
void command(char type, int d0, float data, float coordinate0, float coordinate1, float coordinate2, float coordinate3, float coordinate4, float coordinate5, float spd, float acc)
{
    rb5_goal.type = type;
    rb5_goal.d0 = d0;
    rb5_goal.data = data;
    rb5_goal.coordinate[0] = coordinate0;
    rb5_goal.coordinate[1] = coordinate1;
    rb5_goal.coordinate[2] = coordinate2;
    rb5_goal.coordinate[3] = coordinate3;
    rb5_goal.coordinate[4] = coordinate4;
    rb5_goal.coordinate[5] = coordinate5;
    rb5_goal.spd = spd;
    rb5_goal.acc = acc;
}

//will move the end effector position using the IK on board the RB5
void moveTCP(float coord0, float coord1, float coord2, float theta0, float theta1, float theta2, float spd, float acc)
{
    command('T', 0, 0, coord0, coord1, coord2, theta0, theta1, theta2, spd, acc);
}

void moveTCP(float coord0, float coord1, float coord2, float theta0, float theta1, float theta2)
{
    command('T', 0, 0, coord0, coord1, coord2, theta0, theta1, theta2, .3, .3);
}

//will move each joint individually
void moveJoint(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float spd, float acc)
{
    command('J', 0, 0, theta0, theta1, theta2, theta3, theta4, theta5, spd, acc);
}

void moveJoint(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5)
{
    command('J', 0, 0, theta0, theta1, theta2, theta3, theta4, theta5, .3, .3);
}

//speed should be on a scale of 0-1, where 1 is full speed
void setSpeed(float speed)
{
    command('V', 0, speed,0,0, 0, 0, 0, 0,0,0 );
    ROS_INFO("Changing speed.");
}

//should be used upon starting up the robot to initialize joint encoders
void initialize()
{
    command('I',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Initializing.");
}

//needs to be set! Real mode will ensure the robot moves, whereas simulation
//will still post thetas on the topic, but the robot will not move
void setRealMode(bool realMode)
{
    if(realMode == 1)
    {
      command('R',0,0,0,0, 0, 0, 0, 0,0,0);
      ROS_INFO("Setting to real mode.");
    }
    else
    {
      command('S',0,0,0,0, 0, 0, 0, 0,0,0);
      ROS_INFO("Setting to simulation mode.");
    }
}

//the gripper developed for RB5 will open if we pass in a 1 to the Gripper
//control command.
void openGripper()
{
    //'E' command opens on 1, closes on 2
    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Opening gripper.");

    //will continue to open, so need to enforce a stop on the girpper motors
    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Gripper closed.");
}

//the gripper developed for RB5 will close if we pass in a 2 to the Gripper
//control command
void closeGripper()
{
    //'E' command opens on 1, closes on 2
    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");

    //will continue to open, so need to enforce a stop on the girpper motors
    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Gripper closed.");
}
