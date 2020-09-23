#include "RB5_Client.h"

//dimensions of the tower
const int TABLE_HEIGHT = 132;
const int MAX_Z_OFFSET = 15; //17-2
const int MIN_Z_OFFSET = 0; //2-2
const int MAX_X_OFFSET = 2;
const int MIN_X_OFFSET = 0;

//distance between each block
const float ZOFFSET = 14;
const float XOFFSET = -23.4;

//poke and push distance
const float POKE1OFFSET = -5;
const float POKE2OFFSET = -10;
const float PUSHOFFSET = -100;

float baseX, baseY, baseZ, t0, t1, t2;
float curX, curZ;

void initializeGame(float x, float y, float z, float the0, float the1, float the2)
{
  baseX = x;
  baseY = y;
  baseZ = z;
  t0 = the0;
  t1 = the1;
  t2 = the2;
  return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_ros_client");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("/position_of_object", 1, setCoords);

    actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> ac_("motion", true);
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started.");

    initialize();
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    setRealMode(true);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    setSpeed(0.4);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ROS_INFO("Starting tests.");

    //go to starting position
    moveJoint(-87, 16, 111, 54, -87, 0);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //move to a position
    moveTCP(-15.78, -685.14, 470.7, -179.82, 83.41, -179.82);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //go back home
    moveJoint(-87, 16, 111, 54, -87, 0);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    return 0;
}
