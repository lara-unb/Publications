#include "TransmitterRobot.hpp"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <string>
#include <Conversions.hpp>
#include <cmath>
#include <ledAuxiliarFunctions.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>


TransmitterRobot::TransmitterRobot()
{
    std::cout << "construct" << std::endl;
    //Penalized();
}

TransmitterRobot::~TransmitterRobot()
{
    std::cout << "destruct" << std::endl;
    fromCommsTester.disconnectAll();
}

void TransmitterRobot::Penalized()
{
    std::cout << "Penalized" << std::endl;

    Command command;
    command.leds = LEDAux::changeColor(Conversions::RED, Conversions::OFF, Conversions::OFF);
    //command.body = MotionAux::Stand();
    MotionAux::SendCommand(command);
    while(true)
    {
        if(ButtonPressed())
        {
            break;
        }
    }
}

void TransmitterRobot::manageConnection(int port)
{
    Callibrate();

    Command command;
    command.leds = LEDAux::changeColor(Conversions::RED, Conversions::RED, Conversions::RED);
    //command.body = MotionAux::Stand();
    MotionAux::SendCommand(command);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    std::cout << "manage" << std::endl;

    //command.leds = LEDAux::changeColor(Conversions::OFF, Conversions::OFF, Conversions::OFF);
    //command.body = MotionAux::Stand();
    MotionAux::SendCommand(command);
    //Connect to Receiver

	fromCommsTester.listenOn(port);
	fromCommsTester.waitForIncomingConnection();


}

int TransmitterRobot::readData()
{
    std::cout << "read" << std::endl;

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

void TransmitterRobot::locationChallenge()
{
    std::cout << "Location" << std::endl;


    int readSize;
    int x, y, xOct, yOct, quadrant = 0;
    readSize = fromCommsTester.readData(&packet.payload.location, sizeof(SPLNoWifiLocationPayload));

    x = packet.payload.location.x;
    y = packet.payload.location.y;

    std::cout << "Transmitter robot: received location, x=" << x << ", y=" << y << std::endl;

    Conversions::MoveCenterToCorner(&x, &y);
    Conversions::ConvertToQuadrant(&x, &y, &quadrant);
    xOct = Conversions::decimal_octal(x);
    yOct = Conversions::decimal_octal(y);
    std::vector<int> vec = Conversions::num2vec(xOct, yOct, quadrant);
    for(int i =0; i < 9 ; i++)
    {
        std::cout << vec[i] << std::endl;
    }
    Transmit(vec);
}

void TransmitterRobot::Transmit(std::vector<int> vec)
{
    std::cout << "Transmit" << std::endl;
    for(int i = 0; i<9; i++)
    {
        Conversions::transmitLED(vec[i]);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        Conversions::turnOFF();
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }
    Penalized();
}

bool TransmitterRobot::ButtonPressed()
{
    //std::cout << "Button" << std::endl;
    ButtonData buttons;
    buttons = buttonBoard.load();
    if(buttons.button.pop(1))
    {
        //Chest Button pressed once
        buttonBoard.save(buttons);
        return true;
    }
    buttonBoard.save(buttons);
    return false;
}

void TransmitterRobot::Callibrate()
{
    std::cout << "Calibrate " << std::endl;
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
    for(int color = 2; color<8; color++)
    {
        Conversions::changeChestLed(color);
        boost::this_thread::sleep(boost::posix_time::milliseconds(150));
    }
}
