#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include "lanros2podo.h"

const int OFFSET = 280;

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

        objNumStatic = n.subscribe("/darknet_ros1/found_object", 50, &State::searchStaticCallback, this);
        objPosStatic = n.subscribe("/position_of_object_static", 50, &State::positionStaticCallback, this);
        objNumDynamic= n.subscribe("/darknet_ros2/found_object", 50, &State::searchDynamicCallback, this);
        objPosDynamic = n.subscribe("/position_of_object_dynamic", 50, &State::positionDynamicCallback, this);
        //efPos = n.subscribe("/position_of_endfactor", 1, &State::efCallback, this);
        //ac_.feedback_cb = ac_.SimpleFeedbackCallback();


        //need to initialize the robot before sending commands
        command('I',0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Initializing.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //set mode
        command('R',0,0,0,0, 0, 0, 0, 0,0,0);
        ROS_INFO("Setting to real mode.");
        ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

        //change speed to 0.2
        command('V',0,0.2,0,0, 0, 0, 0, 0,0,0);
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
    ros::Subscriber objNumStatic;
    ros::Subscriber objNumDynamic;
    ros::Subscriber objPosStatic;
    ros::Subscriber objPosDynamic;
    //ros::Subscriber efPos;
    Client ac_;

    int objectNumStatic = 0;
    int objectNumDynamic = 0;
    float objectPointsStatic[3] = {0, 0, 0};
    float objectPointsDynamic[3] = {0, 0, 0};
    //float efPosition[3] = {0, 0, 0};

    void searching();
    void following();
    void objectFound();
    void moveAboveObj(int xMov, int yMov, int zMov);
    void grabObj();
    void openGripper();
    void closeGripper();
    void goHome();
    void pause(float time);
    void resume();

    void searchStaticCallback(const std_msgs::Int8::ConstPtr& msg);
    void positionStaticCallback(const geometry_msgs::Point::ConstPtr& msg);
    void searchDynamicCallback(const std_msgs::Int8::ConstPtr& msg);
    void positionDynamicCallback(const geometry_msgs::Point::ConstPtr& msg);
    void efCallback(const geometry_msgs::Point::ConstPtr& msg);

};

void State::searching()
{
    float xPrev, yPrev, zPrev, xDif, yDif, zDif;
    int contFlag = 0;

    ROS_INFO("Searching.");

    ros::Rate r(20);

    while(ros::ok() && objectNumStatic == 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("There were ", objectNumStatic, " objects found.");

    while(contFlag == 0)
    {
        xPrev = objectPointsStatic[0];
        yPrev = objectPointsStatic[1];
        zPrev = objectPointsStatic[2];

        pause(1);
        ros::spinOnce();

        xDif = xPrev - objectPointsStatic[0];
        yDif = yPrev - objectPointsStatic[1];
        zDif = zPrev - objectPointsStatic[2];

        if(xDif + yDif + zDif < .6 && objectNumStatic > 0)
            contFlag = 1;
    }

    //object found
    std::cout << "Object found using static camera." << std::endl;
    moveAboveObj(objectPointsStatic[0], objectPointsStatic[1], objectPointsStatic[2]);
}

void State::following()
{
    float xMov, yMov, zMov; //xEF, yEF, zEF, distX, distY;

    ros::Rate r(.2);

    r.sleep();

    if(objectNumDynamic == 0)
        goHome();

    while(ros::ok() && objectNumDynamic != 0)
    {
        r.sleep(); //only need for while loop - delete if using if
        ros::spinOnce();

        xMov = objectPointsDynamic[0];
        yMov = objectPointsDynamic[1];
        zMov = objectPointsDynamic[2];

        //xEF = efPosition[0];
        //yEF = efPosition[1];
        //zEF = efPosition[2];

        //distX = std::sqrt((xMov-xEF)*(xMov-xEF));
        //distY = std::sqrt((yMov-yEF)*(yMov-yEF));

        std::cout << "Object found using dynamic camera." << std::endl;
        moveAboveObj(xMov, yMov, zMov);
    }
/*
        //if (dist) < 15, grab
        if(distX < 30 && distY < 45)
        {
            ROS_INFO("Grabbing object");
            grabObj();
        }
        else
/*
            [ INFO] [1564928796.518929923]: Starting again.
            distX = 314 and distY = 370
            [ INFO] [1564928802.204582798]: Following object
            distX = 325 and distY = 374
            [ INFO] [1564928805.685346169]: Following object
            distX = 319 and distY = 375
            [ INFO] [1564928808.739869340]: Following object
            distX = 323 and distY = 375
            [ INFO] [1564928811.694783952]: Following object
            distX = 321 and distY = 375
            [ INFO] [1564928814.599267932]: Following object
            distX = 323 and distY = 376
            [ INFO] [1564928817.454152948]: Following object

            std::cout << "distX = " << distX << " and distY = " << distY << std::endl;
            ROS_INFO("Following object");
            moveAboveObj(xMov, yMov, zMov);

        r.sleep();
    }

    else
*/
        goHome();
    //if object isn't detected or ros closes, go home

}

void State::moveAboveObj(int xMov, int yMov, int zMov)
{
    zMov = zMov + OFFSET;

    command('T',0,0, xMov, yMov, zMov, 90, 0, 90,0.8,0.3);
    std::cout << "xMov = " << xMov << " yMov = " << yMov << " and zMov - OFFSET = " << zMov - 280 << std::endl;
    //ROS_INFO("Moving to (", xMov, ", ", yMov, ", ", zMov, ")");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    following();
}

void State::grabObj()
{
    ROS_INFO("Object grabbed. Going home.");
    goHome();
}


/*
    zMov = zMov - OFFSET;

    command('T',0,0, xMov, yMov, zMov, 90, 0, 90,0.8,0.3);
    ROS_INFO("Lowering to object.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    closeGripper();

    //pick up object
    command('T',0,0,-15.78, -685.14,470.7,-179.82,83.41,-179.82,0.8,0.3);
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
*/

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
    command('T',0,0,-118.02, -429.46, 484.6, 90, 0, 90,0.8,0.3);
    ROS_INFO("Going home.");
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

//    closeGripper();

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

void State::searchStaticCallback(const std_msgs::Int8::ConstPtr& msg)
{
    objectNumStatic = msg->data;
}

void State::searchDynamicCallback(const std_msgs::Int8::ConstPtr& msg)
{
    objectNumDynamic = msg->data;
}

void State::positionStaticCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    objectPointsStatic[0] = msg->x;
    objectPointsStatic[1] = msg->y;
    objectPointsStatic[2] = msg->z;
}

void State::positionDynamicCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    objectPointsDynamic[0] = msg->x;
    objectPointsDynamic[1] = msg->y;
    objectPointsDynamic[2] = msg->z;
}
/*
void State::efCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    efPosition[0] = msg->x;
    efPosition[1] = msg->y;
    efPosition[2] = msg->z;

    std::cout << "efPos[0] = " << efPosition[0] << " efPos[1] = " << efPosition[1] << "efPos[2] = " << efPosition[2] << std::endl;
}
*/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rb5_ros_client");
    State newState;

    return 0;
}
