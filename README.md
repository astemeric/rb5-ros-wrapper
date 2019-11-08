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
