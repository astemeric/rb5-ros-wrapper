#include "RBCobot/RBCobot.h"

int main(int argc, char **argv) {


    printf("Press enter to continue. Then, please connect to ROS Action Server...\n");
    getchar();
    RBCobot rbCobot;
    rbCobot.start();
    usleep(1000000);

    printf("Connected...\n");

    printf("Press any key to exit\n");
    getchar();

    rbCobot.release();
    return 0;
}
