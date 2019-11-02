#include "lanpodo2ros.h"

LANPODO2ROS::LANPODO2ROS()
{
    size = sizeof(message);
    buffer = new char[size];
    sock = 0;
}

LANPODO2ROS::~LANPODO2ROS()
{
    delete [] buffer;
    buffer = nullptr;
}
