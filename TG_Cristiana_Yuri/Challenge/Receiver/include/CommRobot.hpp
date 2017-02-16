#pragma once
#include <vector>
#include <string>
#include <TcpConnection.h>

class CommRobot
{
public:
    CommRobot(std::string remoteIP, int remotePort);
    virtual ~CommRobot();
    void SendLocationToCommTester(int x, int y);
    void SendDataToCommTester(std::vector<int> dataVec, int offset);
    void Connect();
    void Disconnect();


private:
    // additional offset between end of one fragment and start of next
    // if 0, then fragments follow each other normally
    // if negative then fragments will overlap
    // if positive then fragments will leave gaps so that not all of original
    // message is sent
    // must be within the range of +/- (SIM_NO_WIFI_PAYLOAD_SIZE - sizeof(SimNoWifiDataPayloadHeader))
    int testingFragmentPrevToNextOffset;
    //Connection with the Communication tester program
    TcpConnection toCommsTester;
    std::string remoteIP;
    int remotePort;
};
