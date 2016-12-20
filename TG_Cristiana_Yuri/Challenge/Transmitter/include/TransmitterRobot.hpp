#pragma once
#include "TcpConnection.h"
#include "SPLNoWifiChallenge.h"
#include "SimNoWifiPacket.h"


class TRobot
{
	public:
		TRobot();
        ~TRobot();
        void manageConnection(int);
        int readData();
        void locationChallenge();

    private:
        SPLNoWifiPacket packet;
        TcpConnection fromCommsTester;
};
