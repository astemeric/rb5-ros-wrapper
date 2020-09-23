#include "RB5_Client.h"

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
