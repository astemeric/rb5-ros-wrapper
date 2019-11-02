#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <rb5_ros_wrapper/MotionAction.h>
#include "lanros2podo.h"
#include <sensor_msgs/JointState.h>
#include <errno.h>
#include <rb5_ros_wrapper/rx.h>

#define IPAddr "127.0.0.9"

const int PORT = 4000;

int sock = 0;
struct sockaddr_in ROSSocket;

LANROS2PODO TX;
int testFlag = 0;

///delete after testing

//std::chrono::high_resolution_clock::time_point prevClock;
//std::chrono::high_resolution_clock::time_point curClock;
//std::chrono::nanoseconds duration;

///

bool connect()
{
    //this is really ugly, I know. Going to fix in the future.
    //connect to RB5 command thread
    if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error creating socket \n");
        return false;
    }

    ROSSocket.sin_family = AF_INET;
    ROSSocket.sin_port = htons(PORT);

    if(inet_pton(AF_INET, IPAddr, &ROSSocket.sin_addr)<=0)
    {
        printf("\n Invalid Address \n");
        return false;
    }

    if(connect(sock, (struct sockaddr *)&ROSSocket, sizeof(ROSSocket)) < 0)
    {
        printf("\n Connection failed. Error = %s \n", strerror(errno));
        return false;
    }

    printf("\n Client connected to server! \n");

    return true;
}

class MotionAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rb5_ros_wrapper::MotionAction> as_;
    ros::Subscriber objectSub = nh_.subscribe("/rb5_RX", 50, &MotionAction::updateCallback, this);

    std::string action_name_;

    rb5_ros_wrapper::MotionFeedback feedback_;
    rb5_ros_wrapper::MotionResult result_;
    rb5_ros_wrapper::rx rx_msg;

public:

    MotionAction(std::string name) :
        as_(nh_, name, boost::bind(&MotionAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~MotionAction(void)
    {
    }

    void writeTX(const rb5_ros_wrapper::MotionGoalConstPtr &goal)
    {
        //char *buffed = new char[TX.size];
        void *buffed;
        //send over the TX motion data
        TX.command.type = goal->type;

        if(feedback_.mode != "Simulation")
        {
            if(goal->d0 == 1)
            {
                TX.command.d0 = 0;
                TX.command.d1 = 1;
            }
            else if(goal->d0 == 2)
            {
                TX.command.d0 = 1;
                TX.command.d1 = 0;
            }
            else
            {
                TX.command.d0 = 0;
                TX.command.d1 = 0;
            }
        }
        else
        {
            TX.command.d0 = 0;
            TX.command.d1 = 0;
        }

        TX.command.data = goal->data;
        for(int i = 0; i < 6; i++)
        {
            TX.command.coordinate[i] = goal->coordinate[i];
        }
        TX.command.spd = goal->spd;
        TX.command.acc = goal->acc;

        buffed = (void*)malloc(TX.size);
        memcpy(buffed, &TX.command, TX.size);

        //delete after testing
        //auto prevClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        //std::cout << prevClock << ", ";
        //////////
        send(sock, buffed, TX.size, 0);

        free(buffed);
        buffed = nullptr;

        return;
    }

    //only called when client requests goal
    void executeCB(const rb5_ros_wrapper::MotionGoalConstPtr &goal)
    {
        //RB5 runs at 100 Hz
        ros::Rate r(100);
        bool success = true;

//        ROS_INFO("%s: Received Motion %i\n");

        ros::Time beginTime = ros::Time::now();
        ros::Time curTime;
        ros::Duration durTime;


        //===write to RB5===

        int rxDoneFlag = 0;
        int activeFlag = 0;

        //write TX to RB5

        writeTX(goal);

        //loop until TX complete

        while(rxDoneFlag == 0)
        {

            curTime = ros::Time::now();
            durTime = curTime - beginTime;
//            std::cout << "durTime = " << durTime << std::endl;
//            std::cout << "rxDoneFlag = " << rxDoneFlag << std::endl;
            //std::cout << "rxDoneFlag = " << rxDoneFlag << std::endl;
            //std::cout << "activeFlag = " << activeFlag << std::endl;

            //check that preempt has not been requested by client
            if(as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

//            std::cout << "TX.command.type = " << TX.command.type << std::endl;

            switch(goal->type)
            {
                case 'I':

                    if(durTime.toSec() > 10)
                    {
                        rxDoneFlag = 1;
                    }
                    break;

                case 'J':
              //      std::cout << "activeFlag = " << activeFlag << std::endl;
              //      std::cout << "rxDoneFlag = " << rxDoneFlag << std::endl;
              //      std::cout << "ROSState = " << feedback_.state << std::endl;
              //      std::cout << "Actual Robot State = " << RX.message.robot_state << std::endl;

                    if(activeFlag == 0 && feedback_.state == "Moving")
                    {
                        activeFlag = 1;
                    }
                    /*else if(feedback_.state == "Idle")
                    {
                        if(durTime.toSec() > 5)
                        {
                            rxDoneFlag = 1;
                        }
                    }*/
                    else if(feedback_.state == "Idle" && activeFlag == 1)
                    {
                        rxDoneFlag = 1;
                    }
                    break;

                case 'T':
                //    std::cout << "activeFlag = " << activeFlag << std::endl;
                //    std::cout << "rxDoneFlag = " << rxDoneFlag << std::endl;
                //    std::cout << "ROSState = " << feedback_.state << std::endl;
                //    std::cout << "Actual Robot State = " << RX.message.robot_state << std::endl;

                    if(activeFlag == 0 && feedback_.state == "Moving")
                    {
                        activeFlag = 1;
                    }
                    /*else if(feedback_.state == "Idle")
                    {
                        if(durTime.toSec() > 5)
                        {
                            rxDoneFlag = 1;
                        }
                    }*/
                    else if(feedback_.state == "Idle" && activeFlag == 1)
                    {
                        rxDoneFlag = 1;
                    }
                    break;

                case 'E':
                    if(durTime.toSec() < 0.7)
                    {
                        break;
                    }
					else
						rxDoneFlag = 1;
                case 'F':
                    if(durTime.toSec() > 0.7)
                    {
                        rxDoneFlag = 1;
                    }
                    break;
                case 'P':
                    rxDoneFlag = 1;
                    break;
                case 'Q':
                    rxDoneFlag = 1;
                    break;
                default:
                    if(durTime.toSec() > 3)
                    {
                        rxDoneFlag = 1;
                    }
                    break;
            }

            //std::cout << "Serverstatus = " << returnServerStatus();

            if(rx_msg.state == 3)
            {
                feedback_.state = "Moving";
                //delete after testing
                //if(testFlag == 0)
                //{
                //    auto curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                //    std::cout << curClock << std::endl;
                //    testFlag = 1;
                //}
                ///
            }
            else if(rx_msg.state == 2)
                feedback_.state = "Paused or Collision";
            else
            {
                if(testFlag == 1)
                {
                   testFlag = 0;
                }

                feedback_.state = "Idle";
            }
            if(rx_msg.mode == 0)
                feedback_.mode = "Real";
            else
                feedback_.mode = "Simulation";

            feedback_.spd = rx_msg.spd;

            for(int i = 0; i < 6; i++)
            {
                feedback_.coordinate[i] = rx_msg.coordinate[i];
                feedback_.endEffectorPos[i] = rx_msg.endEffectorPos[i];
            }

            feedback_.endEffector = rx_msg.endEffector;

            if(returnServerStatus())
            {
                as_.publishFeedback(feedback_);
            }
            //publishResult();

            //maintain desired loop rate

            r.sleep();
            ros::spinOnce();
        }

        if(success)
        {
            //ROS_INFO("%s: Succeeded");
            result_.state = feedback_.state;
            result_.mode = feedback_.mode;
            result_.spd = feedback_.spd;
            for(int i = 0; i < 6; i++)
            {
                result_.coordinate[i] = feedback_.coordinate[i];
                result_.endEffectorPos[i] = feedback_.endEffectorPos[i];
            }

            result_.endEffector = feedback_.endEffector;

            as_.setSucceeded(result_);
        }

    }

    int returnServerStatus()
    {
        if(as_.isActive())
            return 1;
        else
            return 0;
    }

    void updateCallback(const rb5_ros_wrapper::rx::ConstPtr& msg)
    {
        //std::cout << "Publishing Feedback" << std::endl;

        rx_msg.state = msg->state;
        rx_msg.mode = msg->mode;
        rx_msg.spd = msg->spd;
        rx_msg.endEffector = msg->endEffector;
        for(int i = 0; i < 6; i++)
        {
          rx_msg.coordinate[i] = msg->coordinate[i];
          rx_msg.endEffectorPos[i] = msg->endEffectorPos[i];
        }

        //printf("Not stuck on read\n");
        //printf("size of message = %d\n", sizeof(RX.message));;
    }
};



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "rb5_ros_wrapper");

    if(connect() == false)
    {
        printf("\n\n Failed to connect. Closing...\n");
        return -1;
    }

    //valread = read(sock, buffer, 1024);
    //printf("%s\n", buffer);
    ROS_INFO("Starting Action Server");
    MotionAction motion("motion");

    ros::spin();

    return 0;
}
