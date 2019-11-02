#ifndef LANROS2PODO_H
#define LANROS2PODO_H


class LANROS2PODO
{
public:
    LANROS2PODO();
    ~LANROS2PODO();

    int size;
    int sock;
    char *buffer;
    struct Motion
    {
        char type;
        int d0;
        int d1;
        float data;
        float coordinate[6];
        float spd;
        float acc;
    };

    Motion command;

//bool requestJoints(bool request);

};

#endif // LANROS2PODO_H
