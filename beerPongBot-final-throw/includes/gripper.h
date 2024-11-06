#ifndef GRIPPER_H
#define GRIPPER_H

#include <string>

class Gripper
{
private:
    int socketId;
    void sendCommand (std::string command);
    std::string receiveResponse ();
public:
    Gripper (const std::string& ip, int port);
    ~Gripper ();
    void home ();
    void open (bool wait = true);
    void grip ();
    void flushResponse ();
};

#endif