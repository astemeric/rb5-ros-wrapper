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
#include "lanpodo2ros.h"
#include <sensor_msgs/JointState.h>
#include <errno.h>
#include <rb5_ros_wrapper/rx.h>

const int PORT = 4001;

int sock = 0, opt = 1, new_socket;
struct sockaddr_in ROSSocket;
int sockLen = sizeof(ROSSocket);
int testFlag = 0;

LANPODO2ROS RX;

bool connect()
{
  printf("\n Waiting to connect... \n");

  //connect to RB5 update thread
  if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
      printf("\n Error creating socket \n");
      return false;
      }

      if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,&opt, sizeof(opt)))
      {
        perror("setsockopt");
        exit(EXIT_FAILURE);
      }

      ROSSocket.sin_family = AF_INET;
      ROSSocket.sin_addr.s_addr = INADDR_ANY;
      ROSSocket.sin_port = htons(PORT);

      if (bind(sock, (struct sockaddr *)&ROSSocket, sizeof(ROSSocket))<0)
      {
        perror("bind failed");
        exit(EXIT_FAILURE);
      }

      if (listen(sock, 3) < 0)
      {
        perror("listen");
        exit(EXIT_FAILURE);
      }

      if ((new_socket = accept(sock, (struct sockaddr *)&ROSSocket,(socklen_t*)&sockLen))<0)
      {
        perror("accept");
        exit(EXIT_FAILURE);
      }

      printf("\n Update connected! \n");

      return true;
}

int main(int argc, char *argv[])
{

    if(connect() == false)
    {
      printf("\n\n Failed to connect. Closing...\n");
      return -1;
    }

    ros::init(argc, argv, "rb5_ros_update");

    ros::NodeHandle nh_;

    ros::Publisher rx_pub = nh_.advertise<rb5_ros_wrapper::rx>("rb5_RX", 1);
    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("rb5_joint_state", 1);

    ros::Rate r(100);
    rb5_ros_wrapper::rx rx_msg;
    sensor_msgs::JointState joint_state;

      printf("\n\n Listening and Publishing...\n");

    while(ros::ok())
    {
      joint_state.name.resize(6);
      joint_state.position.resize(6);

      //printf("Stuck on read?\n");
      if(recv(new_socket, RX.buffer, RX.size, MSG_DONTWAIT) > 0)
      {
        memcpy(&RX.message, RX.buffer, RX.size);

        //delete after testing
        //if(RX.message.robot_state == 3 && testFlag == 0)
        //{
        //  auto curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        //  std::cout << "receivedTime = " << curClock << std::endl;
        //  testFlag = 1;
        //}
        //else if(RX.message.robot_state == 1 && testFlag == 1)
        //{
        //  testFlag = 0;
        //}
        //////////

        rx_msg.state = RX.message.robot_state;
        rx_msg.spd = RX.message.speed;

        for(int i = 0; i < 6; i++)
        {
            rx_msg.coordinate[i] = RX.message.joint_angles[i];
            rx_msg.endEffectorPos[i] = RX.message.tcp_position[i];

            //the below switch statement is just to post to jointState.msg
            switch(i){
              case 0:
                joint_state.name[i] = "base_joint";
                joint_state.position[i] = rx_msg.coordinate[i];
                break;
              case 1:
                joint_state.name[i] = "shoulder_joint";
                joint_state.position[i] = rx_msg.coordinate[i];
                break;
              case 2:
                joint_state.name[i] = "elbow_joint";
                joint_state.position[i] = rx_msg.coordinate[i];
                break;
              case 3:
                joint_state.name[i] = "wrist_joint";
                joint_state.position[i] = rx_msg.coordinate[i];
                break;
              case 4:
                joint_state.name[i] = "wrist_joint2";
                joint_state.position[i] = rx_msg.coordinate[i];
                break;
              case 5:
                joint_state.name[i] = "wrist_link3";
                joint_state.position[i] = rx_msg.coordinate[i];
            }
        }

        rx_msg.endEffector = RX.message.end_effector;
      }

      rx_pub.publish(rx_msg);
      joint_pub.publish(joint_state);

      r.sleep();
      ros::spinOnce();
    }

    return 0;
}
