# rb5-ros-wrapper

Authors: Jason Kreitz, University of Nevada Las Vegas;
         Giho Jang, Rainbow Robotics

The use of the ros wrapper requires a physical connection to an RB5.
<br />

------------------------------------------------------------------------------------------------------------------------------------

To install:
Download, place into ~PATH/catkin_ws/src, then catkin_make

Before using:
Make sure that your IP address is on the 10.0.2.x subnet (ie. Your IP address starts with 10.0.2.x and the gateway is 10.0.2.1) - the robot IP is 10.0.2.7 if you need to ping it... <br />

Also, make sure that the robot is turned on, and the E-Stop is off... <br />
To turn the robot on, rotate the "on" switch clockwise until it is horizontal to the floor. Then press the blue reset button to connect the power... <br />
You will know the robot is on if the lights flash... <br />

------------------------------------------------------------------------------------------------------------------------------------


To run:
Run the following rosrun commands in separate terminals in order:

roscore <br />
rosrun rb5_ros_wrapper cobot_controller <br />
rosrun rb5_ros_wrapper rb5_update <br />
rosrun rb5_ros_wrapper rb5_ros_wrapper <br /> <br />
Wait for the cobot_controller to say "Connected..." - then run: <br />

rosrun rb5_ros_wrapper RB5_Client <br />

<br />
<br />
WARNING: The RB5_Client script will need to be restarted anytime any of the other nodes are closed. This is to reset the home position, so it does not move erratically.
<br />
<br />
<br />


------------------------------------------------------------------------------------------------------------------------------------

To use:
The RB5_Client ROS node contains the function calls that the end user can use to send commands to the RB5. 
<br />
<br/>

To use example clients:
Move the desired client file in the example_clients folder to the src/src folder.  
Edit the CMakeLists under add_executable(RB5_Client src/RB5_Client.cpp) - change the RB5_Client.cpp to the new file. Then ROS RUN
         Note: The way that CMakeLists works is (rosnode_name src/file_to_compile)
         rosnode_name: The name that ROS will use reference the file AFTER running catkin_make
         file_to_compile: The name of the file before compilation
         
Note: Any time you edit the client script, you will need to run catkin_make again...
<br />
<br />
How to Create Custom Client:
Use the examples as a reference. The RB5_Client.h header file will have the definitions for all the function calls.
<br />
<br />
For a demonstration of the API function calls, look at example_clients/RB5_Jenga.cpp
<br />
<br />

------------------------------------------------------------------------------------------------------------------------------------

<br />
A further explanation of how this wrapper works:
<br />
<br />
<br />

User Application

       This application uses the ROS Action Client API. You can see an example file at src/RB5_Client.cpp
        You can see examples of using the RB5 client at line 87.
            I send a "J" command (for "move all joints") to a ROS Action Server(within the src/wrapper.cpp file) that acts as an intermediary between ROS and the RB5.


ROS Communication Nodes

    Next we have our ROS intermediary nodes. These just handle communication between the User Application and the RB5. These are at src/wrapper.cpp and src/update.cpp.
        Wrapper.cpp will listen to the ROS Action Client and send the data to the RB5 application on your computer.
        Update will listen to the RB5 application on your computer and send the data to the ROS Action Client
        In all honesty, I should have just had them both in one file using threads, but the Action Client API made things a bit more confusing than I was expecting.

RB5

    The primary logic for the RB5 function calls will be in src/cobotAPI/CobotController.h
        This file is an intermediate application between ROS and the Robot Controller.
        This file is the closest that we can get to the robot. Everything else is handled on board the controller.
            You can see the function calls that occur when your application receives a command packet from ROS (around line 348).
            You can see function calls such as moveJoint, moveTCP (move end effector), etc (around line 525). 
            You can see how commands are sent to the controller as well (around line 476).

<br />


------------------------------------------------------------------------------------------------------------------------------------

Caution: If you receive the following error when running catkin_make

         #include rb5_ros_wrapper/MotionAction.h

Then follow these steps:
<br />
<br />
In the CMakeLists.txt, comment out all add_executable and target_link_libraries lines using a # symbol. Run catkin_make once, then uncomment those lines and run catkin_make again.
<br />
<br />
catkin_make is multithreaded by default, and it will normally try to compile the executables before generating custom messages. This will lead to compilation errors, because it is searching for a header file (MotionAction.h) that has not yet been generated. MotionAction.h contains all the custom action messages for the Action Client/Server, so by running catkin_make the first time (with the executables commented), it will generate the messages. The second catkin_make will then run smoothly, because all the message header files have been generated.
