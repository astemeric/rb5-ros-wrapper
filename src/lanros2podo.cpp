#include "lanros2podo.h"

LANROS2PODO::LANROS2PODO()
{
    size = sizeof(command);
    buffer = new char[size];
    sock = 0;
}

LANROS2PODO::~LANROS2PODO()
{
    delete [] buffer;
    buffer = nullptr;
}

/*bool LANROS2PODO::requestJoints(bool request)
{
    if(request == true)
    {
        this->command.jointRequest = true;
        return command.jointRequest;
    }
    else
    {
        this->command.jointRequest = false;
        return command.jointRequest;
    }
}
*/
