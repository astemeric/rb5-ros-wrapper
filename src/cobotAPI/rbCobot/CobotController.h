#ifndef COBOT_CONTROLLER_H
#define COBOT_CONTROLLER_H


#include "TCPClient.h"

#include <iostream>
#include <stdarg.h>  // For va_start, etc.
#include <memory>    // For std::unique_ptr
#include <mutex>          // std::mutex

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

#define _USE_MATH_DEFINES



#define MAX_SKIP_UPDATE 10
#define DegToRad(a) ((a) * static_cast<float>(M_PI)/180.0f)

#define MAX_MC              6
#define RT_TIMER_PERIOD_MS  2
#define COMMAND_CANID           0x01

#define MAX_SHARED_DATA 116
#define MAX_CONFIG_DATA 24

#include <memory>
#include <iostream>
#include <string>
#include <cstdio>

//delete after testing//

//std::chrono::high_resolution_clock::time_point prevClock;
//std::chrono::high_resolution_clock::time_point curClock;
//std::chrono::nanoseconds duration;
int testFlag = 0;

///////////////////////

typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    FRC:1;		// Friction Compensation //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;   // Multi-Turn Error //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    CAN:1;
        unsigned    rsvd2:7;
    }b;
    unsigned char B[4];
}mSTAT;


enum RCR_INIT_ERR_FLAG{
    INIT_STAT_ERR_CLEAR = 0,
    INIT_STAT_ERR_NO_SMPS,
    INIT_STAT_ERR_NO_EMG_SW,
    INIT_STAT_ERR_GPIF_PSW,
    INIT_STAT_ERR_GPIF_FET,
    INIT_STAT_ERR_MC_CAN_CHECK,
    INIT_STAT_ERR_MC_FIND_HOME,
    INIT_STAT_ERR_COL_OFFSET,
};

enum RCR_INIT_INFO{
    INIT_STAT_INFO_NOACT = 0,
    INIT_STAT_INFO_VOLTAGE_CHECK,
    INIT_STAT_INFO_DEVICE_CHECK,
    INIT_STAT_INFO_FIND_HOME,
    INIT_STAT_INFO_VARIABLE_CHECK,
    INIT_STAT_INFO_COLLISION_ON,
    INIT_STAT_INFO_INIT_DONE
};




typedef struct{
    char    shared_data_name[64];
    int     shared_data_type;
}shared_data_info;
//extern shared_data_info sd_info_stat[MAX_SHARED_DATA];
//extern shared_data_info sd_info_config[MAX_CONFIG_DATA];

typedef union{
    struct{
        char    header[4];
        // 0
        float   time;                   // time [sec]
        float   jnt_ref[6];             // joint reference [deg]
        float   jnt_ang[6];             // joint encoder value [deg]
        float   cur[6];                 // joint current value [mA]
        // 19
        float   tcp_ref[6];             // calculated tool center point from reference [mm, deg]
        float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg]
        // 31
        float   analog_in[4];           // analog input value of control box [V]
        float   analog_out[4];          // analog output value of control box [V]
        int     digital_in[16];         // digital input value of control box [0 or 1]
        int     digital_out[16];        // digital input value of control box [0 or 1]
        // 71
        float   temperature_mc[6];      // board temperature of each joint [celcius]
        // 77
        int     task_pc;                // (ignore)
        int     task_repeat;            // (ignore)
        int     task_run_id;            // (ignore)
        int     task_run_num;           // (ignore)
        float   task_run_time;          // (ignore)
        int     task_state;             // (ignore)
        // 83
        float   default_speed;          // overriding speed [0~1]
        int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving]
        int     power_state;            // power state
        // 86
        float   tcp_target[6];          // (ignore)
        int     jnt_info[6];            // joint information (look mSTAT)
        // 98
        int     collision_detect_onoff; // collision detect onoff [0:off  1:on]
        int     is_freedrive_mode;      // current freedrive status [0:off  1:on]
        int     program_mode;           // current program mode [0:real mode  1:simulation mode]
        // 101
        int     init_state_info;        // status information of robot initialization process
        int     init_error;             // error code of robot initialization process
        // 103
        float   tfb_analog_in[2];       // analog input value of tool flange board [V]
        int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1]
        int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1]
        float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24]
        // 110
        int     op_stat_collision_occur;
        int     op_stat_sos_flag;
        int     op_stat_self_collision;
        int     op_stat_soft_estop_occur;
        int     op_stat_ems_flag;
        // 115
    }sdata;
    float fdata[MAX_SHARED_DATA];
    int idata[MAX_SHARED_DATA];
}systemSTAT;

typedef union{
    struct{
        char    header[4];
        // 0
        float   sensitivity;            // collision threshold [0~1]
        float   work_x_min;             // (ignore)
        float   work_x_max;             // (ignore)
        float   work_y_min;             // (ignore)
        float   work_y_max;             // (ignore)
        float   work_z_min;             // (ignore)
        float   work_z_max;             // (ignore)
        float   mount_rotate[3];        // direction of gravity [normalized vector]
        // 10
        float   toolbox_size[3];        // virtual collision box size of tool [mm]
        float   toolbox_center_pos[3];  // virtual collision box position of tool [mm]
        float   tool_mass;              // tool mass [kg]
        float   tool_mass_center_pos[3];// center of mass position of tool [mm]
        float   tool_ee_pos[3];         // tool position [mm]
        // 23
    }sdata;
    float fdata[MAX_CONFIG_DATA];
}systemCONFIG;



typedef struct{
    char    header[4];
    char    type;
    char    msg[1000];
    int     len;
}systemPOPUP;

extern int      __IS_PODO_WORKING__;

extern int      _NO_OF_MC;
extern systemSTAT   *sys_status;
extern systemCONFIG *sys_config;
extern systemPOPUP  *sys_popup;
extern systemPOPUP  *sys_custom_alarm;


typedef struct _Pose3D
{
    float x;
    float y;
    float z;

    float rx;
    float ry;
    float rz;

    _Pose3D(float v1,float v2,float v3,float v4,float v5,float v6)
    {
        x = v1;
        y = v2;
        z = v3;

        rx = v4;
        ry = v5;
        rz = v6;
    }
    _Pose3D()
    {

    }

}Pose3D;

typedef float Joint6[6];
typedef struct{
    Joint6 joints;
}JointInfo;


class RBCobot :public Thread
{
    const std::string strType[3] = {"intended","constant","radial"};
    std::mutex mtx;
    int iRequestSkipUpdate;

    TCPClient cmdSocket;
    TCPClient dataSocket;
    ROSUpdate rosUpdate;
    ROSServer rosSocket;

    systemSTAT systemStat;
    systemCONFIG systemConfig;
    systemPOPUP  systemPopup;

    //QByteArray recvBuf;
    std::vector<char> recvBuf;


public:
    bool bPause;

    RBCobot()
    {
        iRequestSkipUpdate = 0;
        bPause = false;
    }
    ~RBCobot()
    {

    }

public:
    bool start()
    {
        memset(&systemStat, 0, sizeof(systemSTAT));
        systemStat.sdata.program_mode = -1;
        systemStat.sdata.robot_state = -1;

        rosSocket.ConnectToClient("127.0.0.9",4000);
        rosUpdate.ConnectToServer("127.0.0.9",4001);

        cmdSocket.setBufferSize(2000);
        cmdSocket.setReceiveEvent([&](TCPSocket *sock,  const unsigned char *data, int){

            std::string str(reinterpret_cast<const char*>(data));
            if(str.compare("The command was executed\n") == 0)
            {
               //client receives validation from the server that command was executed
               //printf("The command was executed!!\r\n");
                mtx.unlock(); //allow updates to the cmdSocket
                iRequestSkipUpdate = MAX_SKIP_UPDATE; //updateRequestSkip to prevent overwriting current command
                systemStat.sdata.robot_state = 3;
            }
        });
        cmdSocket.ConnectToServer("10.0.2.7",5000);

        //note - lambda expressions allow you to remove the need for an explicit slot function call
        //and instead, allow you to pass in the address to the object and run the code in the function call itself
        dataSocket.setBufferSize(2000);
        dataSocket.setReceiveEvent([&](TCPSocket *sock, const unsigned char * data, int len)
        {
            recvBuf.assign(data,data+len);

            while(recvBuf.size() > 4 )
            {
                if( recvBuf[0] == '$')
                {
                    int size = ((int)((unsigned char)recvBuf[2]<<8)|(int)((unsigned char)recvBuf[1]));
                    if(size <= recvBuf.size() )
                    {
                        if(3 == recvBuf[3])
                        {

                            if(iRequestSkipUpdate ==0)
                            {
                                 memcpy(&systemStat,recvBuf.data(),sizeof(systemSTAT));
                            }

                            recvBuf.erase(recvBuf.begin(),recvBuf.begin()+sizeof(systemSTAT));
                            //      printf("systemStat Data Received\r\n");

                        }
                        else if(4 == recvBuf[3])
                        {
                            memcpy(&systemConfig,recvBuf.data(),sizeof(systemCONFIG));
                            recvBuf.erase(recvBuf.begin(),recvBuf.begin()+sizeof(systemCONFIG));
                            //        printf("systemConfig Data Received\r\n");

                        }
                        else if(10 == recvBuf[3])
                        {
                            memcpy(&systemPopup,recvBuf.data(),sizeof(systemPOPUP));
                            recvBuf.erase(recvBuf.begin(),recvBuf.begin()+sizeof(systemPOPUP));
                            ////     printf("systemPopup Data Received\r\n");
                        }
                        else
                        {
                            recvBuf.erase(recvBuf.begin());
                        }
                    }
                    else
                    {
                        return;
                    }

                }
                else
                {
                    recvBuf.erase(recvBuf.begin());
                }
            }
        });

       rosSocket.setReceiveEvent([&](TCPSocket *sock,  const unsigned char *data, int){
           //delete after testing
           //prevClock = std::chrono::high_resolution_clock::now();
           //auto prevClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
           //std::cout << prevClock << ", ";
           //testFlag = 1;
           //printf("prevClock set\n");

           /////////////////////

           memcpy(&rosSocket.data, data, sizeof(rosSocket.data));

            char type = rosSocket.data.type;

            switch(type)
            {
            case 'I':
                initRobot();
                break;
            case 'R':
                setRealMode();
                break;
            case 'S':
                setSimulationMode();
                break;
            case 'J':
                moveJoint(rosSocket.data.coordinate, rosSocket.data.spd, rosSocket.data.acc);
                break;
            case 'T':
                moveTCP(rosSocket.data.coordinate, rosSocket.data.spd, rosSocket.data.acc);
                break;
            case 'V':
                setBaseSpeed(rosSocket.data.data);
                break;
            case 'E':
                toolOut(24, rosSocket.data.d0, rosSocket.data.d1);
                break;
            case 'F':
                toolOut(0, 0, 0);
                break;
            case 'P':
                pauseMotion();
                break;
            case 'H':
                haltMotion();
                break;
            case 'Q':
                resumeMotion();
                break;
            case 'C':
                resumeCollision();
                break;
            case 'W':


            default:
                break;
            }
        });
        rosUpdate.setReceiveEvent([&](TCPSocket *sock, const unsigned char * data, int len){

            rosUpdate.message.robot_state = systemStat.sdata.robot_state;

            rosUpdate.message.program_mode = systemStat.sdata.program_mode;

              for(int i = 0; i < 6; i++)
              {
                  if(rosUpdate.message.program_mode == 1) //if robot is in simulation, send joint reference
                  {
                      rosUpdate.message.joint_angles[i] = systemStat.sdata.jnt_ref[i];
                      rosUpdate.message.tcp_position[i] = systemStat.sdata.tcp_ref[i];
                  }
                  else //else, send encoder values
                  {
                      rosUpdate.message.joint_angles[i] = systemStat.sdata.jnt_ang[i];
                      rosUpdate.message.tcp_position[i] = systemStat.sdata.tcp_ref[i];
                  }
              }

            rosUpdate.message.speed = systemStat.sdata.default_speed;

            rosUpdate.message.end_effector = systemStat.sdata.tfb_voltage_out;

            //printf("spd = %f, mode = %d, and state = %d\n", rosUpdate.message.speed, rosUpdate.message.program_mode, rosUpdate.message.robot_state);
            //printf("size of message = %d\n", sizeof(rosUpdate.message));

            memcpy((void*)data,&rosUpdate.message, sizeof(rosUpdate.message));

            //printf("Stuck on write\n");
            rosUpdate.Write(data, len);
            //printf("Not stuck on write\n");

            //delete after testing

            if(testFlag == 1 && rosUpdate.message.robot_state == 3)
            {
                //curClock = std::chrono::high_resolution_clock::now();
                //duration = (curClock - prevClock);
                //std::cout << duration.count() << std::endl;
                //auto curClock = std::chrono::high_resolution_clock::now().time_since_epoch().count();
                //duration = (curClock - prevClock);

                //std::cout << curClock << std::endl;
                //testFlag = 0;
            }
            ////////////////////////
        });

        bool bRet = dataSocket.ConnectToServer("10.0.2.7",5001);
        if(bRet)
        {
            Thread::start(Thread::HzToSec(100));
        }
        return bRet;

    }
    void pause()
    {
         bPause = true;
    }
    void resume()
    {
        bPause = false;
    }



    template<typename ... Args>
    RBCobot& sendCommand(const std::string& format, Args ... args)
    {
        size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        std::unique_ptr<char[]> buf( new char[ size ] );
        snprintf( buf.get(), size, format.c_str(), args ... );


        std::string str = std::string( buf.get(), buf.get() + size - 1 );

        if(!cmdSocket.isOpen())
        {
            return *this;
        }

        while(bPause == true)
        {
            usleep(1000*100);
        }
        //printf("%s\r\n",str.c_str());
        //std::cout << "str = " << str << std::endl;
        mtx.lock(); //note for tomorrow - mutex never gets unlocked
        int ret =0;
        systemStat.sdata.robot_state = -1; //unknown
        ret = static_cast<int>(cmdSocket.Write(str.c_str())); //write the command to the control box
       return *this;
    }




    // Cobot Control API -------------------
    // <CobotInit>
    // : initialize Cobot
    RBCobot&  initRobot()
    {
         sendCommand("mc jall init");

         return *this;
    }

    // <MoveJoint>
    // : move to target posture in joint coordinate
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot& moveJoint(Joint6 &joint, float spd = -1, float acc = -1)
    {
        return moveJoint(joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],spd,acc);
    }
    RBCobot &moveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd=-1, float acc=-1)
    {

        sendCommand("jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", static_cast<double>(spd),
                    static_cast<double>(acc),
                    static_cast<double>(joint1),
                    static_cast<double>(joint2),
                    static_cast<double>(joint3),
                    static_cast<double>(joint4),
                    static_cast<double>(joint5),
                    static_cast<double>(joint6));
        systemStat.sdata.robot_state = 3; //run
        iRequestSkipUpdate = MAX_SKIP_UPDATE;
        waitUntilMotionDone();
        return *this;
    }

    // <MoveTCP>
    // : move to target posture in cartesian coordinate
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot&  moveTCP(float *pose, float spd = -1, float acc=-1)
    {
        return moveTCP(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5],spd,acc);
    }
    RBCobot&  moveTCP(Pose3D &pose, float spd=-1, float acc=-1)
    {
        return moveTCP(pose.x,pose.y,pose.z,pose.rx,pose.ry,pose.rz,spd,acc);
    }
    RBCobot&  moveTCP(float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1)
    {
        sendCommand("movetcp %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", static_cast<double>(spd),
                    static_cast<double>(acc),
                    static_cast<double>(x),
                    static_cast<double>(y),
                    static_cast<double>(z),
                    static_cast<double>(rx),
                    static_cast<double>(ry),
                    static_cast<double>(rz));
        systemStat.sdata.robot_state = 3; //run
        iRequestSkipUpdate = MAX_SKIP_UPDATE;
        waitUntilMotionDone();
        return *this;
    }

    // <MoveCircle_ThreePoint>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current, first, and final position
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // x1, y1, z1 : first position in mm unit
    // rx1, ry1, rz1 : first orieyawntation (Yaw-Pitch-Roll Euler angle) in degree unit
    // x2, y2, z2 : final position in mm unit
    // rx2, ry2, rz2 : final orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot&  moveCircle_ThreePoint(int type, double x1, double y1, double z1, double rx1, double ry1, double rz1, double x2, double y2, double z2, double rx2, double ry2, double rz2, double spd = -1, double acc = -1)
    {

        assert(type == 0 || type == 1 || type == 2 );
        sendCommand("movecircle threepoints %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    strType[type].c_str(), spd, acc, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2);
        systemStat.sdata.robot_state = 3;
        waitUntilMotionDone();
        return *this;
    }

    // <MoveCircle_Axis>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current position, center position, axis, and rotation angle
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // cx, cy, cz : center position in mm unit
    // ax, ay, az : axis representation (norminal)
    // rot_angle: rotation angle iyawn degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot& moveCircle_Axis(int type, double cx, double cy, double cz, double ax, double ay, double az, double rot_angle, double spd, double acc)
    {
        assert(type == 0 || type == 1 || type == 2 );
        sendCommand("movecircle axis %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                    strType[type].c_str(), spd, acc, rot_angle, cx, cy, cz, ax, ay, az);
        systemStat.sdata.robot_state = 3;
        waitUntilMotionDone();
        return *this;
    }

    // <MoveJointBlend_Clear>
    // : clear joint blend list
    RBCobot& moveJointBlend_Clear()
    {

        return sendCommand("blend_jnt clear_pt");

    }

    // <MoveJointBlend_AddPoint>
    // : add point to the joint blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot&  moveJointBlend_AddPoint(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, double spd = -1, double acc = -1)
    {
        sendCommand("blend_jnt add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);
        systemStat.sdata.robot_state = 3; //run
        waitUntilMotionDone();
        return *this;
    }

    // <MoveJointBlend_MovePoint>
    // : start to move all the points in the joint blend list
    RBCobot&  moveJointBlend_MovePoint()
    {
        sendCommand("blend_jnt move_pt");
        systemStat.sdata.robot_state = 3;
        waitUntilMotionDone();
        return *this;
    }

    // <MoveTCPBlend_Clear>
    // : clear TCP blend list
    RBCobot&  moveTCPBlend_Clear()
    {
        sendCommand("blend_tcp clear_pt");
        return *this;
    }

    // <MoveTCPBlend_AddPoint>
    // : add point to the TCP blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // radius : blend distance in mm unit
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    RBCobot&  moveTCPBlend_AddPoint(double radius, double x, double y, double z, double rx, double ry, double rz, double spd = -1, double acc = -1)
    {
        sendCommand("blend_tcp add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, radius, x, y, z, rx, ry, rz);
        systemStat.sdata.robot_state = 3; //run
        waitUntilMotionDone();
        return *this;
    }


    // <MoveTCPBlend_MovePoint>
    // : start to move all the points in the joint blend list
    RBCobot&  moveTCPBlend_MovePoint()
    {

        sendCommand("blend_tcp move_pt constant");
        systemStat.sdata.robot_state = 3;
        waitUntilMotionDone();
        return *this;
    }

    // <ControlBoxDigitalOut>
    // control digital out ports in control box
    // d0~d15 : digital out value (0 or 1)
    RBCobot&  digitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15)
    {
        sendCommand("digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);
        return *this;
    }

    // <ControlBoxAnalogOut>
    // control analog out ports in control box
    // a0~a3 : analog out value in voltage unit (0~10)
    RBCobot&  analogOut(double a0, double a1, double a2, double a3)
    {
        sendCommand("analog_out %.3f, %.3f, %.3f, %.3f", a0, a1, a2, a3);
        return *this;
    }

    // <ToolOut>
    // control digital out ports and voltage level in tool flange board
    // volt : reference voltage of tool flange board in voltage unit(0, 12, 24)
    // d0, d1 : digital out value (0 or 1)
    RBCobot&  toolOut(int volt, int d0, int d1)
    {
        int temp_volt = volt;
        if((temp_volt != 12) && (temp_volt != 24))
            temp_volt = 0;

        sendCommand("tool_out %d, %d, %d", temp_volt, d0, d1);
        return *this;
    }

    // <ProgramMode_Real>
    // change to 'real robot' mode -- robot will move
    RBCobot& setRealMode()
    {
          sendCommand("pgmode real");
          return *this;
    }

    // <ProgramMode_Simulation>
    // change to 'simulation' mode -- robot will not move except teaching
    RBCobot& setSimulationMode()
    {
         sendCommand("pgmode simulation");
         return *this;
    }

    // <BaseSpeedChange>
    // change base speed -- base speed will be multiplied to motion velocity
    // spd : normalized base speed (0~1)
    RBCobot&  setBaseSpeed(double spd)
    {

        if(spd > 1.0)
        {
            spd = 1.0;
        }
        if(spd < 0.0)
        {
            spd = 0.0;
        }

        sendCommand("sdw default_speed %.3f", spd);
        return *this;
    }

    // <MotionPause>
    // pause the current motion
    RBCobot&  pauseMotion()
    {

        sendCommand("task pause");
        return *this;
    }

    // <MotionHalt>
    // halt the current motion
    // !! CAUTION : user would better escape the motion sequence
    //            : if not, the next motion will be activated immediately
    RBCobot&  haltMotion()
    {

        sendCommand("task stop");
        return *this;
    }

    // <MotionResume>
    // resume the paused motion
    RBCobot& resumeMotion()
    {

        sendCommand("task resume_a");
        return *this;
    }

    // <CollisionResume>
    // resume the motion which is paused due to external collision
    RBCobot& resumeCollision()
    {
        sendCommand("task resume_b");
        return *this;
    }
    // -------------------------------------

    bool isMotionIdle()
    {
        //if we are waiting for valid updates from control box, continue waiting
        if(iRequestSkipUpdate >0)
        {
       //     printf("IsMotionIdle : false\r\n");
            return false;
        }
        //else if robot is idle, finish waiting
      //  printf("IsMotionIdle : %d\r\n",(systemStat.sdata.robot_state == 1) || (systemStat.sdata.robot_state == 0) );
        return ((systemStat.sdata.robot_state == 1) || (systemStat.sdata.robot_state == 0)  );

    }


    void getPose(Pose3D & pose)
    {
        pose.x =  systemStat.sdata.tcp_pos[0];
        pose.y =  systemStat.sdata.tcp_pos[1];
        pose.z =  systemStat.sdata.tcp_pos[2];
        pose.rx =  systemStat.sdata.tcp_pos[3];
        pose.ry =  systemStat.sdata.tcp_pos[4];
        pose.rz =  systemStat.sdata.tcp_pos[5];
    }
    void getJoint(Joint6 & jnt)
    {
        for(int i = 0 ; i < 6 ; i++)
        {
            jnt[i] = systemStat.sdata.jnt_ang[i];
        }
    }


    // RBThread interface

    Pose3D getPose()
    {
        Pose3D pose;
        pose.x =  systemStat.sdata.tcp_pos[0];
        pose.y =  systemStat.sdata.tcp_pos[1];
        pose.z =  systemStat.sdata.tcp_pos[2];
        pose.rx =  systemStat.sdata.tcp_pos[3];
        pose.ry =  systemStat.sdata.tcp_pos[4];
        pose.rz =  systemStat.sdata.tcp_pos[5];
        return pose;
    }
     RBCobot& waitUntilMotionDone()
    {
        while(!isMotionIdle())
        {
            //std::cout << "robotStatus = " << systemStat.sdata.robot_state << std::endl;
            //std::cout << "taskStatus = " << systemStat.sdata.task_state << std::endl;

            usleep(10);
        }
         return *this;
    }
    void getRefJoint(Joint6 &jnt)
    {
        for(int i = 0 ; i < 6 ; i++)
        {
            jnt[i] = systemStat.sdata.jnt_ref[i];
        }
    }
    void getRefPose(Pose3D &pose)
    {
        pose.x =  systemStat.sdata.tcp_ref[0];
        pose.y =  systemStat.sdata.tcp_ref[1];
        pose.z =  systemStat.sdata.tcp_ref[2];
        pose.rx =  systemStat.sdata.tcp_ref[3];
        pose.ry =  systemStat.sdata.tcp_ref[4];
        pose.rz =  systemStat.sdata.tcp_ref[5];
    }


    RBCobot&  relativeMoveLocalTCP(float offset_x, float offset_y, float offset_z)
    {
        Pose3D pose;
        getRefPose(pose);

        Eigen::AngleAxisf R(DegToRad(pose.rx), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf P(DegToRad(pose.ry), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf Y(DegToRad(pose.rz), Eigen::Vector3f::UnitZ());

        Eigen::Vector3f vecOffset(offset_x,offset_y,offset_z);
        Eigen::Matrix3f matRot = Y.matrix() * P.matrix()*R.matrix();
        vecOffset = matRot *vecOffset;

        pose.x = pose.x + vecOffset[0];
        pose.y = pose.y + vecOffset[1];
        pose.z = pose.z + vecOffset[2];

        moveTCP(pose);
        return *this;
    }

    RBCobot&  relativeMoveTCP(float offset_x, float offset_y, float offset_z, float offset_roll=0, float offset_pitch=0, float offset_yaw=0,float spd = -1, float acc= -1)
    {
        Pose3D curPose;
        getRefPose(curPose);
        curPose.x +=offset_x;
        curPose.y +=offset_y;
        curPose.z +=offset_z;

        curPose.rx +=offset_roll;
        curPose.ry +=offset_pitch;
        curPose.rz +=offset_yaw;
        moveTCP(curPose,spd,acc);
        return *this;
    }
    float getRadius()
    {
        Pose3D pose;
        getRefPose(pose);
        float mag = sqrtf((pose.x*pose.x)+(pose.y*pose.y));
        return mag;
    }


protected:
    void onInit()
    {

    }
    void onRelease()
    {
        cmdSocket.release();
        dataSocket.release();
    }
    void onUpdate(double deltaTime)
    {
        //std::cout << "robotStatus = " << systemStat.sdata.robot_state << std::endl;
        //std::cout << "taskStatus = " << systemStat.sdata.task_state << std::endl;
        if(iRequestSkipUpdate >0)
        {
            iRequestSkipUpdate--;
            return;
        }
        if(dataSocket.isOpen())
        {
            dataSocket.Write("reqcfg");
            dataSocket.Write("reqdata");
        }
    }

    // Thread interface
public:
    void release()
    {

    };
};


#endif
