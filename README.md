# rb5-ros-wrapper

Authors: Jason Kreitz, University of Nevada Las Vegas;
         Giho Jang, Rainbow Robotics

The use of the ros wrapper requires a physical connection to an RB5.

To install:
Download, place into ~PATH/catkin_ws/src, then catkin_make

To use:
The client ROS node contains the function calls that the end user can use to send commands to the RB5. Edit that and catkin_make

To run:
Compile the main.cpp in src/CobotAPI. Then run.

rosrun rb5_ros_wrapper rb5_update
rosrun rb5_ros_wrapper RB5_Client
rosrun rb5_ros_wrapper rb5_ros_wrapper


A further explanation of how this wrapper works:

User Application
    This application uses the ROS Action Client API. You can see an example file at src/RB5_Client.cpp
        You can see examples of using the RB5 client at line 87.
            I send a "J" command (for "move all joints") to a ROS Action Server(within the src/wrapper.cpp file) that acts as an intermediary between ROS and the RB5.


ROS Communication Nodes

    Next we have our ROS intermediary nodes. These just handle communication between the User Application and the RB5. These are at src/wrapper.cpp and src/update.cpp.
        Wrapper.cpp will listen to the ROS Action Client and send the data to the RB5 application on your computer.
        Update will listen to the RB5 application on your computer and send the data to the ROS Action Client


In all honesty, I should have just had them both in one file using threads, but the Action Client API made things a bit more confusing than I was expecting. I should have just used regular Publisher/Subscriber communication, but KAIST said they wanted the Action Client API...

RB5

    The primary logic for the RB5 function calls will be in src/cobotAPI/CobotController.h
        This file is an intermediate application between ROS and the Robot Controller.
        This file is the closest that we can get to the robot. Everything else is handled on board the controller.
            You can see the function calls that occur when your application receives a command packet from ROS (around line 348).
            You can see function calls such as moveJoint, moveTCP (move end effector), etc (around line 525). 
            You can see how commands are sent to the controller as well (around line 476).
