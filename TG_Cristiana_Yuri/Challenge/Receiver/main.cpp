#include <iostream>
#include <fstream>
#include <sstream>
#include <cerrno>
#include <string>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alvalue/alvalue.h>

#include <ReceiverRobot.hpp>
#include <TransmitterRobot.hpp>

void parseOpt(std::string *receiverTransmitter, std::string *remoteIP, int *remotePort, std::string *naoBrokerIP, int *naoBrokerPort, int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Print help mesages.")
        ("mode,m", po::value<std::string>(receiverTransmitter)->default_value("T"), "Robot Mode. T for Transmitter R for Receiver. Default: Receiver")
        ("rip", po::value<std::string>(remoteIP)->default_value("10.2.0.1"), "IP of the comm tester. Default: ")
        ("rport", po::value<int>(remotePort)->default_value(10100),"Port of the parent broker. Default: 38000")
        ("pip", po::value<std::string>(naoBrokerIP)->default_value("127.0.0.1"), "IP of the parent broker. Default: 127.0.0.1")
        ("pport", po::value<int>(naoBrokerPort)->default_value(9559),"Port of the parent broker. Default: 9559");
    po::positional_options_description positionalOptions;
    po::variables_map vm; // Map containing all the options with their values
    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

        if (vm.count("help"))
        {
            std::cout << "SPL No Wifi Challenge."                                   << std::endl;
            std::cout << "Can be executed locally or remotelly."                   << std::endl;
            std::cout                                                              << std::endl;
            std::cout << "Usage:"                                                  << std::endl;
            std::cout << "  " << argv[0] << " [option] [input] "                  << std::endl;
            std::cout                                                              << std::endl;
            std::cout << desc                                                      << std::endl;

            exit(0);
        }

        po::notify(vm);
    }
    catch(po::error &e)
    {
        std::cerr << e.what() << std::endl;
        std::cout << desc << std::endl;
        exit(1);
    }
}

// ==========================================================================

int main(int argc, char **argv)
{
//    std::cout << "sizeof SPLCommsChallengeHeader is " << sizeof(SPLNoWifiHeader) << "  bytes\n";
//    std::cout << "sizeof SPLCommsChallengePacket is " << sizeof(SPLNoWifiPacket) << "  bytes\n";
//    std::cout << "sizeof SPLLocationPayload is " << sizeof(SPLNoWifiLocationPayload) << "  bytes\n";
//    std::cout << "sizeof SPLDataPayloadHeader is " << sizeof(SPLNoWifiDataPayloadHeader) << "  bytes\n";
//    std::cout << "sizeof SPLDataPayload is " << sizeof(SPLNoWifiDataPayload) << "  bytes\n";

    // check if we are the transmitter robot or the receiver robot


    std::string parentBrokerIP;
    int parentBrokerPort;
    std::string receiverTransmitter;
    std::string remoteIP;
    int remotePort;

    setlocale(LC_NUMERIC, "C"); // Need this to for SOAP serialization of floats to work

    parseOpt(&receiverTransmitter, &remoteIP, &remotePort, &parentBrokerIP, &parentBrokerPort, argc, argv);

    std::cout << "parse created..." << std::endl;

    // A broker needs a name, an IP and a port:
    const std::string brokerName = "mybroker";
    // FIXME: would be a good idea to look for a free port first
    int brokerPort = 0;
    // listen port of the broker (here an anything)
    const std::string brokerIp = "";

    // Create your own broker
    boost::shared_ptr<AL::ALBroker> broker;
    try
    {
        broker = AL::ALBroker::createBroker(
            brokerName,
            brokerIp,
            brokerPort,
            parentBrokerIP,
            parentBrokerPort,
            0    // you can pass various options for the broker creation,
                 // but default is fine
        );
    }
    catch(const AL::ALError& e)
    {
        std::cerr << "Fail to connect broker to: "
                  << parentBrokerIP
                  << ":"
                  << parentBrokerPort
                  << std::endl
                  << e.what()
                  << std::endl;

        AL::ALBrokerManager::getInstance()->killAllBroker();
        AL::ALBrokerManager::kill();

        return 1;
    }

    //Create the proxy to access the motion module
    boost::shared_ptr<AL::ALProxy> Motion;

    try
    {
        Motion = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker,"Motion"));
        std::cout << "proxy created..." << std::endl;
    }
    catch (const AL::ALError& e)
    {
        std::cerr << "Could not create proxy: " << e.what() << std::endl;
        return 1;
    }

    try
    {
        Motion->callVoid("StartLoop");
    }
    catch (const AL::ALError& e)
    {
        std::cerr << e.what() << std::endl;
    }

    if(std::toupper(receiverTransmitter[0]) == 'R')
    {
        ReceiverRobot receiver(remoteIP, remotePort);
        bool flagStop = false;
        int currentState = State::PENALIZED;
        while(!flagStop)
        {
            switch (currentState) {
            case State::PENALIZED:
                currentState = receiver.Penalized();
                break;
            case State::START:
                currentState = receiver.Start();
                break;
            case State::CALIBRATE:
                currentState = receiver.Calibrate();
                break;
            case State::SYNC:
                currentState = receiver.Sync();
                break;
            case State::WAIT:
                currentState = receiver.Wait();
                break;
            case State::RECEIVE:
                currentState = receiver.Receive();
                break;
            case State::SEND:
                currentState = receiver.Send();
                break;
            default:
                break;
            }
        }
    }
    else if(std::toupper(receiverTransmitter[0]) == 'T')
    {
        int type;
        TransmitterRobot robot;
        std::cout << "OK" << std::endl;
        robot.manageConnection(remotePort);
        while(true)
        {
            type = robot.readData();
            if(type == 1)
            {
                robot.locationChallenge();
            }
            if(type == 2)
            {
                //data stuff
            }
        }

    }


    /*const char * receiverTransmitter = argv[1];
    const char * listeningPort = argv[2];
    const char * remoteIP = argv[3];
    const char * remotePort = argv[4];*/

    /*if (toupper(receiverTransmitter[0]) == 'R')
        receiverRobot(atoi(listeningPort), remoteIP, atoi(remotePort));
    else
        transmitterRobot(atoi(listeningPort), remoteIP, atoi(remotePort));
*/
    return(0);
}
