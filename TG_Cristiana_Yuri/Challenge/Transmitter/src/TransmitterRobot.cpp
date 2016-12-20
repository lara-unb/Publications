#include "TransmitterRobot.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>
#include <string>





TRobot::TRobot()
{

}

TRobot::~TRobot()
{
    fromCommsTester.disconnectAll();
}

void TRobot::manageConnection(int port)
{

    //Connect to Receiver

	fromCommsTester.listenOn(port);
	fromCommsTester.waitForIncomingConnection();


}

int TRobot::readData()
{

    int readSize, tipo;
    readSize =  fromCommsTester.readData(&packet.header, sizeof(SPLNoWifiHeader));

    std::cout << "Tipo:" ;
    tipo = (int) packet.header.type;
    std::cout << tipo << std::endl;
    if(packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION)
    {
        return 1;
    }
    else if (packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_DATA)
    {
        return 2;
    }
    return 0;
}

void TRobot::locationChallenge()
{
    int readSize;
    readSize = fromCommsTester.readData(&packet.payload.location, sizeof(SPLNoWifiLocationPayload));
    std::cout << "Transmitter robot: received location, x=" << packet.payload.location.x << ", y=" << packet.payload.location.y << std::endl;

}
