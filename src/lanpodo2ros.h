#ifndef LANPODO2ROS_H
#define LANPODO2ROS_H


class LANPODO2ROS
{
public:
    LANPODO2ROS();
    ~LANPODO2ROS();
    int size;
    int sock;
    char *buffer;

    struct Update{
        int robot_state;
        int program_mode;
        float speed;
        float joint_angles[6];
        float tcp_position[6];
        float end_effector;
    };

    Update message;
};

#endif // LANPODO2ROS_H
