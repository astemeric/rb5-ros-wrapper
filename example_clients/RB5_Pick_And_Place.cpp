#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "lanros2podo.h"

const int OFFSET = 70;

typedef actionlib::SimpleActionClient<rb5_ros_wrapper::MotionAction> Client;

////THIS IS NOT DONE. I STILL NEED TO IMPLEMENT COLLISION CHECKS AND SINGULARITY CHECKS
////Note ctrl-c will not kill the node
////Also, please make sure that you close the action client after you exit the action server
////The Robot will need to initialize again


class State
{
public:

    State() : ac_("motion", true)
    {

        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");

        objectSub = n.subscribe("/number_of_objects", 50, &State::searchCallback, this);
        objectPos = n.subscribe("/position_of_object", 50, &State::positionCallback, this);

        //need to initialize the robot before sending commands
        command('I',0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Initializing.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //set mode
        command('R',0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Setting to real mode.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //change speed to 0.6
        command('V',0,0.6,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Changing speed.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //set to home
        goHome();
    }

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

private:

    rb5_ros_wrapper::MotionGoal rb5_goal;
    ros::NodeHandle n;
    ros::Subscriber objectSub;
    ros::Subscriber objectPos;
    Client ac_;


    int objectNum = 0;
    float objectPoints[3] = {0, 0, 0};

    void searching();
    void searchCallback(const std_msgs::Int32::ConstPtr& msg);
    void positionCallback(const geometry_msgs::Point::ConstPtr& msg);
    void objectFound();
    void openGripper();
    void closeGripper();
    void goHome();
    void pause(float time);
    void resume();
};

void State::searching()
{
    float xPrev, yPrev, zPrev, xDif, yDif, zDif;
    int contFlag = 0;

    ros::Rate r(40);

    while(ros::ok() && objectNum == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("There were ", objectNum, " objects found.");

    while(contFlag == 0)
    {
        xPrev = objectPoints[0];
        yPrev = objectPoints[1];
        zPrev = objectPoints[2];

        pause(1);
        ros::spinOnce();

        xDif = xPrev - objectPoints[0];
        yDif = yPrev - objectPoints[1];
        zDif = zPrev - objectPoints[2];

        if(xDif + yDif + zDif < .6 && objectNum > 0)
            contFlag = 1;
    }

    objectNum = 0;
    objectFound();
}

void State::searchCallback(const std_msgs::Int32::ConstPtr& msg)
{
    objectNum = msg->data;
}

void State::positionCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    objectPoints[0] = msg->x;
    objectPoints[1] = msg->y;
    objectPoints[2] = msg->z;
}

void State::objectFound()
{
    float xMov, yMov, zMov, xDest, yDest, zDest;

    xMov = objectPoints[0];
    yMov = objectPoints[1];
    zMov = objectPoints[2] + OFFSET;

    xDest = 806.04;
    yDest = 0;
    zDest = 50 + OFFSET;

    //this will assume stationary object.
    //If object moves, there is no check in place

    //object found

    openGripper();

    //move to the object

    command('T',0,0, xMov, yMov, zMov, 90, 0, 90,0.8,0.3);
    ROS_INFO("Moving to (", xMov, ", ", yMov, ", ", zMov, ")");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    zMov = zMov - OFFSET;

    command('T',0,0, xMov, yMov, zMov, 90, 0, 90,0.8,0.3);
    ROS_INFO("Lowering to object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    closeGripper();

    //pick up object
    command('T',0,0,-15.78, -685.14,470.7,-170.82,83.41,-179.82,0.8,0.3);
    ROS_INFO("Starting transport.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //move towards the destination
    command('T',0,0, xDest, yDest, zDest, 180, -90, -90,0.8,0.3);
    ROS_INFO("Moving towards destination.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    zDest -= OFFSET;

    command('T',0,0, xDest, yDest, zDest, 180, -90, -90,0.8,0.3);
    ROS_INFO("Placing at destination");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    openGripper();

    zDest += OFFSET;

    command('T',0,0, xDest, yDest, zDest, 180, -90, -90,0.8,0.3);
    ROS_INFO("Object placed.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    goHome();
}

void State::openGripper()
{
    //'E' command opens on 1, closes on 2
    command('E',1,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Opening gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //will continue to open, so need to force a stop
    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Gripper closed.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

void State::closeGripper()
{
    //'E' command opens on 1, closes on 2
    command('E',2,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Closing gripper.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    //will continue to close, so need to force a stop
    command('F',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Gripper closed.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

void State::goHome()
{
    command('J',0,0,-31.07, -20.94, 122.16, 78.74, -30.99, -90.99,0.8,0.3);
    ROS_INFO("Going home.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    closeGripper();

    searching();
}

void State::pause(float time)
{
    command('P',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Waiting ", time, " seconds.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    ros::Time beginTime = ros::Time::now();
    ros::Time curTime;
    ros::Duration durTime;
    int continueFlag = 0;

    while(continueFlag == 0)
    {
        curTime = ros::Time::now();
        durTime = curTime - beginTime;

        if(durTime.toSec() > time)
            continueFlag = 1;
    }

    resume();
}

void State::resume()
{
    command('Q',0,0,0,0, 0, 0, 0, 0,0,0);
    ROS_INFO("Starting again.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_ros_client");
    State newState;

    return 0;
}
