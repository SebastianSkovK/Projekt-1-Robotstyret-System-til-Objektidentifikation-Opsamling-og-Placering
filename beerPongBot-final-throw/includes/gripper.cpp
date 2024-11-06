#include "gripper.h"
#include <iostream>
#include <stdexcept>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

Gripper::Gripper (const std::string& ip, int port)
{
    std::cout << "Connecting to gripper..." << std::endl;

    socketId = socket (AF_INET, SOCK_STREAM, 0);

    if (socketId == -1)
        throw std::logic_error ("Failed to create socket");

    auto in_addr = inet_addr (ip.c_str ());

    if (in_addr == INADDR_NONE)
        throw std::logic_error ("Invalid IP address");

    sockaddr_in sockaddr_in;
    sockaddr_in.sin_addr.s_addr = in_addr;
    sockaddr_in.sin_family = AF_INET;
    sockaddr_in.sin_port = htons(port);

    if (connect (socketId, (sockaddr*)&sockaddr_in, sizeof (sockaddr_in)) == -1)
        throw std::logic_error ("Failed to connect to gripper");

    home();

    sendCommand ("VERBOSE=1");
    receiveResponse();

    std::cout << "Gripper initialized." << std::endl;
}

Gripper::~Gripper ()
{
    sendCommand ("BYE()");

    close (socketId);

}

void Gripper::sendCommand (std::string command)
{
    command += "\n";

    if (send (socketId, command.c_str (), command.size (), 0) == -1)
        throw std::logic_error ("Failed to send command to gripper");
}

std::string Gripper::receiveResponse ()
{
    char buffer[1024];
    int received = recv (socketId, buffer, sizeof (buffer), 0);

    if (received == -1)
        throw std::logic_error ("Failed to receive response from gripper");

    auto response = std::string (buffer, received - 1);

    std::cout << response << std::endl;
    return response;
}

void Gripper::home ()
{
    sendCommand ("HOME()");

    if (receiveResponse () != "ACK HOME")
        throw std::logic_error ("Failed to receive gripper home acknowledgement");

    if (receiveResponse () != "FIN HOME")
        throw std::logic_error ("Failed to home gripper");
}

void Gripper::open (bool wait)
{
    sendCommand ("RELEASE()");

    if (wait)
    {
        if (receiveResponse () != "ACK RELEASE")
            throw std::logic_error ("Failed to receive gripper release acknowledgement");

        if (receiveResponse () != "FIN RELEASE")
            throw std::logic_error ("Failed to release gripper");

        std::cout << "Gripper opened." << std::endl;
    }
}

void Gripper::flushResponse ()
{
    receiveResponse ();
}

void Gripper::grip ()
{
    sendCommand ("GRIP(5, 35)");

    if (receiveResponse () != "ACK GRIP")
        throw std::logic_error ("Failed to receive gripper grip acknowledgement");

    if (receiveResponse () != "FIN GRIP")
        throw std::logic_error ("Failed to grip gripper");

    std::cout << "Gripper closed." << std::endl;
}