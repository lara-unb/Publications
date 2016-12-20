#include <CommRobot.hpp>
#include <SPLNoWifiChallenge.h>
#include <SimNoWifiPacket.h>

CommRobot::CommRobot(std::string remoteIP, int remotePort) : testingFragmentPrevToNextOffset(0), remoteIP(remoteIP), remotePort(remotePort)
{
}

CommRobot::~CommRobot()
{
    toCommsTester.disconnectAll();
}

void CommRobot::SendLocationToCommTester(int x, int y)
{
    SPLNoWifiPacket packet;
    packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION;
    packet.payload.location.x = (int16_t) x;
    packet.payload.location.y = (int16_t) y;

    int nSent = toCommsTester.sendData(&packet, sizeof(SPLNoWifiHeader) + sizeof(SPLNoWifiLocationPayload));
    if(nSent == -1)
    {
        std::cerr << "Problem sending location to comms tester" << std::endl;
    }
}

void CommRobot::SendDataToCommTester(std::vector<int> dataVec, int offset)
{
    SPLNoWifiPacket packet;
    // now build the message for the comms tester

    // first fill in and copy the overall packet header
    packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_DATA;
    packet.payload.data.header.fragmentOffset = (uint16_t)offset;
    packet.payload.data.header.fragmentLength = (uint16_t)dataVec.size();

    for(uint i=0; i < dataVec.size(); i++)
    {
        packet.payload.data.data[i] = (uint8_t) dataVec[i];
    }

    int nSent = toCommsTester.sendData(&packet, sizeof(SPLNoWifiHeader) + sizeof(SPLNoWifiLocationPayload));
    if(nSent == -1)
    {
        std::cerr << "Problem sending location to comms tester" << std::endl;
    }


}

void CommRobot::Connect()
{
    toCommsTester.connectTo(remoteIP, remotePort);
}

void CommRobot::Disconnect()
{
    toCommsTester.disconnectAll();
}
