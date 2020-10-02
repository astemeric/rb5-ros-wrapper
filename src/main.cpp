#include "cobotAPI/rbCobot/RBCobot.h"

int main(int argc, char **argv) {


    printf("Press enter to continue. Then, please connect to ROS Action Server...\n");
    getchar();
    RBCobot rbCobot;
    rbCobot.start();

    printf("Press any key to exit\n");
    getchar();

    rbCobot.release();
    return 0;
}
