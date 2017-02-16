#pragma once
#include "TcpConnection.h"
#include "SPLNoWifiChallenge.h"
#include "SimNoWifiPacket.h"

#include <unBoard.hpp>
#include <ButtonData.hpp>


class TransmitterRobot
{
	public:
        TransmitterRobot();
        ~TransmitterRobot();
        void manageConnection(int);
        int readData();
        void locationChallenge();

    private:
        SPLNoWifiPacket packet;
        TcpConnection fromCommsTester;
        unBoard<ButtonData> buttonBoard;
        bool ButtonPressed();
        void Penalized();
        void Transmit(std::vector<int>);
        void Callibrate();
};
