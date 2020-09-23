#include "RB5_Client.h"

//note if you want to close out of the program if there is a command time-out
//replace the line 
//    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
//with the following two lines
//    ac.sendGoal(goal);
//    bool finished_before_timeout = ac.waitForResult
//then you can use the finished_before_timeout variable for flow control

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

    //starting position
    moveJoint(-87, 16, 111, 54, -87, 0);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));


    /***** start the game here ******/
    float x, y, z, t0, t1, t2;

    std::cout << "Please enter the center of block (2,0) in the form: x y z t0 t1 t2" << std::endl;
    std::cin >> x >> y >> z >> t0 >> t1 >> t2;

    initializeGame(x, y, z, t0, t1, t2);

    if(z < TABLE_HEIGHT)
    {
      std::cout << "z is too low, exiting" << std::endl;
      return -1;
    }

    std::cout << "Starting Game" << std::endl;

    bool continueGame = 1;
    int blockX = 0, blockZ = 2, newY;
    float xCoord, zCoord;

    xCoord = baseX;
    zCoord = baseZ;

    moveTCP(xCoord, baseY, zCoord, t0, t1, t2);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    curX = xCoord;
    curZ = zCoord;

    std::cout << "Moved to " << blockZ << ", " << blockX << std::endl;

    while(continueGame == 1)
    {
      bool poke;
      std::cout << "Enter row number: ";
      std::cin >> blockZ;
      std::cout << "Enter column number: ";
      std::cin >> blockX;

      //because we start at 2, we need to make sure 2 is the base.
      blockZ -= 2;

      //move to position
      if(blockX < MIN_X_OFFSET || blockX > MAX_X_OFFSET || blockZ < MIN_Z_OFFSET || blockZ > MAX_Z_OFFSET)
      {
          std::cout << "Cannot move to this position" << std::endl;
          std::cout << "x = " << blockX << " and z = " << blockZ << ". Make sure these are between "
                    << MIN_X_OFFSET << ", " << MAX_X_OFFSET << " and " << MIN_Z_OFFSET << ", " << MAX_Z_OFFSET
                    << " respectively." << std::endl;
      }
      else
      {
          xCoord = blockX * XOFFSET;
          xCoord += baseX;
          zCoord = blockZ * ZOFFSET;
          zCoord += baseZ;

          moveTCP(xCoord, baseY, zCoord, t0, t1, t2);
          ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

          curX = xCoord;
          curZ = zCoord;

          std::cout << "Moved to " << blockZ + 2 << ", " << blockX << std::endl;
      }

      std::cout << "Poke? (0/1; 0 = N, 1 = Y)" << std::endl;
      std::cin >> poke;

      if(poke == 1)
      {
          //poke the block
          newY = baseY + POKE1OFFSET;
          moveTCP(curX, newY, curZ, t0, t1, t2);
          ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
          moveTCP(curX, baseY, curZ, t0, t1, t2);
          ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
          std::cout << "Block poked." << std::endl;

          std::cout << "Poke Again? (0/1; 0 = N, 1 = Y)" << std::endl;
          std::cin >> poke;

          if(poke == 1)
          {
              newY = baseY + POKE2OFFSET;
              moveTCP(curX, newY, curZ, t0, t1, t2);
              ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
              moveTCP(curX, baseY, curZ, t0, t1, t2);
              ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
              std::cout << "Block poked." << std::endl;

              std::cout << "Push? (0/1; 0 = N, 1 = Y)" << std::endl;
              std::cin >> poke;
              if(poke == 1)
              {
                  newY = baseY + PUSHOFFSET;
                  moveTCP(curX, newY, curZ, t0, t1, t2);
                  ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
                  moveTCP(curX, baseY, curZ, t0, t1, t2);
                  ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));
                  std::cout << "Block pushed." << std::endl;
              }
          }
      }

      {
          std::cout << "Continue? (0/1; 0 = N, 1 = Y)" << std::endl;
          std::cin >> continueGame;
          if(continueGame == 1)
          {
            std::cout << "Continuing..." << std::endl;
          }
          else
          {
            std::cout << "Exiting..." << std::endl;
          }
      }
    }

    //go home
    moveJoint(-87, 16, 111, 54, -87, 0);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    moveJoint(-83, 16, 111, 54, -87, 0);
    ac_.sendGoalAndWait(rb5_goal, ros::Duration(10.0));

    return 0;
}
