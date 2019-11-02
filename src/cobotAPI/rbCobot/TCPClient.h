#ifndef RBCLIENT_H
#define RBCLIENT_H


#include "Thread.h"
#include <unordered_map>
#include <sys/epoll.h>
#include <functional>
#include <bits/stl_vector.h>


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <string>
#include <vector>

#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <netinet/tcp.h>
#include <vector>



#define MAXEVENTS 64

class TCPSocket
{
    //int sockFD;
    sockaddr_in servAddr;
    std::vector<unsigned char>rBuf;
    std::string address;
    int port;


public:
    int sockFD;
    TCPSocket()
    {
        connected_addr[0] = 0;
        connected_port = 0;
        connect_stat = 0;
    }
    ~TCPSocket()
    {

    }

    bool createServer(int port)
    {

        sockFD = socket(AF_INET, SOCK_STREAM, 0);

        memset(&servAddr, '0', sizeof(servAddr));
        servAddr.sin_family = AF_INET;
        servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        servAddr.sin_port = htons(port);

        int optval = 1;
        if(setsockopt(sockFD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1){
            return false;
        }
        if(setsockopt(sockFD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1){
            return false;
        }


        if(::bind(sockFD, (struct sockaddr*)&servAddr, sizeof(servAddr)) ==-1)
        {
            return false;
        }

        return true;
    }
    TCPSocket* accept()
    {
        sockaddr in_addr;

        socklen_t in_len = sizeof(in_addr);


        int ret = ::accept(sockFD, (struct sockaddr*)&in_addr, &in_len);
        if(ret == -1)
        {
            return nullptr;
        }
        TCPSocket * clientSocket = new TCPSocket();
        clientSocket->sockFD =ret;
        return clientSocket;
    }
    const char* getAddressString()
    {
        return address.c_str();
    }
    int getPort()
    {
        return port;
    }


    bool connect(const char * address,  uint16_t port)
    {
        if(connect_stat == 1)
        {
            if((strcmp(address, connected_addr) == 0) && (port == connected_port))
            {
                return true;
            }
            else
            {
                close();
            }
        }

        if((sockFD = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            return false;
        }

        memset(&servAddr, '0', sizeof(servAddr));

        servAddr.sin_family = AF_INET;
        servAddr.sin_port = htons(port);

        if(inet_pton(AF_INET, address, &servAddr.sin_addr)<=0)
        {
            close();
            return false;
        }

        int optval = 1;
        if(setsockopt(sockFD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1){
            return false;
        }
        if(setsockopt(sockFD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1){
            return false;
        }

        setsockopt(sockFD, IPPROTO_TCP, TCP_NODELAY, (char *) &optval, sizeof(int));

        int synRetries = 2; // Send a total of 3 SYN packets => Timeout ~7s
        setsockopt(sockFD, IPPROTO_TCP, TCP_SYNCNT, &synRetries, sizeof(synRetries));


        if(::connect(sockFD, (struct sockaddr*)&servAddr, sizeof(servAddr)) == -1){
            return false;
        }



        sprintf(connected_addr, "%s", address);
        connected_port = port;
        connect_stat = 1;
        return true;
    }
    void disconnect()
    {

    }
    void close()
    {
        ::shutdown(sockFD, SHUT_RD);
        ::close(sockFD);
        sockFD = 0;
        connected_addr[0] = 0;
        connected_port = 0;
        connect_stat = 0;
    }
    bool setNonBlock()
    {
        int flags;

        flags = fcntl(sockFD, F_GETFL, 0);
        if (flags == -1)
        {
            return  false;
        }

        flags |= O_NONBLOCK;
        if (fcntl(sockFD, F_SETFL, flags) == -1)
        {
            return false;
        }

        return true;
    }

    unsigned int write(const unsigned char * data,int len)
    {
        // return ::write(sockFD,data,len);
        return send(    sockFD, data, len, MSG_NOSIGNAL );
    }

    unsigned int write(std::string &str)
    {
        // return ::write(sockFD,str.c_str(),str.size());
        return send(    sockFD, str.c_str(),str.size(), MSG_NOSIGNAL );
    }
    unsigned int write(const char * str)
    {
        int len = strlen(str);
        // return ::write(sockFD,str,len);
        return send(    sockFD, str, len, MSG_NOSIGNAL );

    }
    unsigned int read(unsigned char * data,int len, char process)
    {
        return ::read(sockFD,data,len);
    }

    int getSockFD()
    {
        return sockFD;
    }

    int connect_stat;   //1: connected  0: disconnected
    char connected_addr[256];
    uint16_t connected_port;
};




class TCPClient : public Thread
{
public:
    TCPClient()
    {
        setBufferSize(500);
        bConnected = false;
    }
    ~TCPClient()
    {

    }

    struct Update{
        int robot_state;
        int program_mode;
        float speed;
        float joint_angles[6];
        float tcp_position[6];
        float end_effector;
    } message;

    bool isOpen()
    {
        return bConnected;
    }
    bool ConnectToServer(const char *addr, unsigned int port)
    {

        bConnected = clientSocket.connect(addr, port);
        if(bConnected)
        {
            start();
        }

        return bConnected;

    }
    void setReceiveEvent(std::function<void(TCPSocket *sock,  const unsigned char *data,int count)> fnReceive)
    {
        this->fnReceive = fnReceive;
    }
    void setBufferSize(size_t s)
    {
        bufSize = s;
        recvBuff.resize(bufSize);
    }

    void close()
    {

        clientSocket.close();
        bConnected = false;
    }
    void release()
    {
        close();
        Thread::release();
        Join();
    }

    unsigned int Write(const unsigned char *data, int len)
    {
        return clientSocket.write(data,len);
    }
    unsigned int Write(std::string &str)
    {
        return clientSocket.write(str);

    }
    unsigned int Write(const char *str)
    {
        return clientSocket.write(str);
    }

protected:

    bool bConnected;
    size_t bufSize;
    std::vector<unsigned char> recvBuff;
    std::function<void (TCPSocket *, const unsigned char *, int)> fnReceive;
    TCPSocket clientSocket;

    void onInit()
    {

    }
    void onRelease()
    {

    }
    virtual void onUpdate(double deltaTime)
    {

        size_t rzLen = clientSocket.read(recvBuff.data(),static_cast<int>(bufSize), 'c');

        if(rzLen >0)
        {
            if(fnReceive != nullptr)
            {
                fnReceive(&clientSocket,recvBuff.data(),static_cast<int>(rzLen));
            }
        }
    }

};

class ROSUpdate : public TCPClient
{
public:
    ROSUpdate()
    {
        setBufferSize(sizeof(message));
        bConnected = false;
    }

    void onUpdate(double deltaTime)
    {
        if(fnReceive != nullptr)
        {
            fnReceive(&clientSocket,recvBuff.data(),static_cast<int>(bufSize));
        }
    }

};

class ROSServer : public Thread
{

    bool bConnected;
    size_t recBufSize;
    std::vector<unsigned char> recvBuff;
    std::function<void (TCPSocket *, const unsigned char *, int)> fnReceive;
    TCPSocket newSocket, *serverSocket;
public:

    struct command{
        char type;
        int d0;
        int d1;
        float data;
        float coordinate[6];
        float spd;
        float acc;
    } data;

    ROSServer()
    {
        setRecBufferSize(sizeof(data));
        //setSendBufferSize(sizeof(message));
        //setRecBufferSize(500);
        //setSendBufferSize(500);
        bConnected = false;

    }
    ~ROSServer()
    {

    }

    bool isOpen()
    {
        return bConnected;
    }
    bool ConnectToClient(const char *addr, unsigned int port)
    {
        if(newSocket.createServer(port) == -1)
        {
            return false;
        }

        if(::listen(newSocket.getSockFD(), 5) == -1)
        {
            return false;
        }

        serverSocket = newSocket.accept();

        if(serverSocket == nullptr)
        {
            return false;
        }

        //bConnected = serverSocket->connect(addr, port); NOTE, SERVER CAN NOT USE CONNECT, WHICH IS RESERVED FOR CLIENTS
        start();

        return bConnected;

    }
    void setReceiveEvent(std::function<void(TCPSocket *sock,  const unsigned char *data,int count)> fnReceive)
    {
        this->fnReceive = fnReceive;
    }

    void setRecBufferSize(size_t s)
    {
        recBufSize = s;
        recvBuff.resize(recBufSize);
    }

    void close()
    {
        newSocket.close();
        serverSocket->close();
        serverSocket = nullptr;
        bConnected = false;
    }
    void release()
    {
        close();
        Thread::release();
        Join();
    }

    unsigned int Write(unsigned char *data, int len)
    {
        return serverSocket->write(data,len);
    }
    unsigned int Write(std::string &str)
    {
        return serverSocket->write(str);

    }
    unsigned int Write(const char *str)
    {
        return serverSocket->write(str);
    }

protected:

    void onInit()
    {

    }
    void onRelease()
    {

    }
    void onUpdate(double deltaTime)
    {
        size_t rzLen = serverSocket->read(recvBuff.data(),static_cast<int>(recBufSize), 'r');
        if(rzLen >0)
        {
            if(fnReceive != nullptr)
            {
                fnReceive(serverSocket,recvBuff.data(),static_cast<int>(rzLen));
            }
        }
    }
};
/*
class ROSUpdate : public TCPClient
{

    bool bConnected;
    size_t sendBufSize;
    std::vector<unsigned char> sendvBuff;
    std::function<void (TCPSocket *, unsigned char *, int)> fnSend;
    TCPSocket rosSocket;
public:


    ROSUpdate()
    {
        //setRecBufferSize(sizeof(data));
        setSendBufferSize(sizeof(message));
        //setSendBufferSize(500);
        bConnected = false;

    }
    ~ROSUpdate()
    {

    }

    bool isOpen()
    {
        return bConnected;
    }
    bool ConnectToServer(const char *addr, unsigned int port)
    {

        bConnected = rosSocket.connect(addr, port);
        if(bConnected)
        {
            start();
        }

        return bConnected;

    }

    void setSendEvent(std::function<void(TCPSocket *sock,  unsigned char *data,int count)> fnSend)
    {
        this->fnSend = fnSend;
    }

    void setSendBufferSize(size_t s)
    {
        sendBufSize = s;
        sendvBuff.resize(sendBufSize);
    }

    void close()
    {
        rosSocket.close();
        bConnected = false;
    }
    void release()
    {
        close();
        Thread::release();
        Join();
    }

    unsigned int Write(unsigned char *data, int len)
    {
        return rosSocket.write(data,len);
    }
    unsigned int Write(std::string &str)
    {
        return rosSocket.write(str);

    }
    unsigned int Write(const char *str)
    {
        return rosSocket->write(str);
    }

protected:

    void onInit()
    {

    }
    void onRelease()
    {

    }
    void onUpdate(double deltaTime)
    {
        //will send updates to ROS at 100 Hz
        if(fnSend != nullptr)
        {
            fnSend(serverSocket, sendvBuff.data(), static_cast<int>(sendBufSize));
        }
    }
};
*/
#endif // RBCLIENT_H
