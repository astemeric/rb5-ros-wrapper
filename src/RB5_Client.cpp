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
float xPos, yPos, zPos;

//std::chrono::high_resolution_clock::time_point prevClock;
//std::chrono::high_resolution_clock::time_point curClock;
//std::chrono::nanoseconds duration;

/*
void command(char type, float data, float coordinate[], float spd, float acc)
{
    rb5_goal.type = type;
    rb5_goal.data = data;
    for(int i = 0; i < 6; i++)
        rb5_goal.coordinate[i] = coordinate[i];
    rb5_goal.spd = spd;
    rb5_goal.acc = acc;
}*/
//rb5_ros_wrapper::MotionGoal
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

void setCoords(const geometry_msgs::Point::ConstPtr &msg)
{
    xPos = msg->x;
    yPos = msg->y;
    zPos = msg->z;
}

int main(int argc, char *argv[])
{

    float xMov, yMov, zMov;

    ros::init(argc, argv, "rb5_ros_client");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/position_of_object", 1, setCoords);

    actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> ac_("motion", true);
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    command('I',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Initializing.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::Duration(1.0).sleep();

    command('R',0,0,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Setting to simulation mode.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::Duration(1.0).sleep();


    command('V',0,0.35,0,0, 0, 0, 0, 0,0,0);
    //ROS_INFO("Changing speed.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ROS_INFO("Starting tests.");

    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Sending goal.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -685.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -585.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -685.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

/*
    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Sending goal.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,-15.78, -685.14,    470.7,     -179.82,      83.41, -179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


    command('T',0,0,806.04,     -0.01,    120.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Dropping towards box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('T',0,0,806.04,     -0.01,    352.00,     180,      -90,      -90,0.8,0.3);
    ROS_INFO("Moving away from box.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Action server ended, returning home.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
*/
//for(int i = 0; i < 50; i++)
//{
//    auto prevClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << prevClock << ", ";
//
//    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
//    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//
//    auto curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << curClock << std::endl;
//
//
//    prevClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << prevClock << ", ";
//
//    command('J',0,0,-32.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
//    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//
//    curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << curClock << std::endl;
//
//
//    prevClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << prevClock << ", ";
//
//    command('J',0,0,-33.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
//    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//
//    curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    std::cout << curClock << std::endl;
//}

    return 0;
}
