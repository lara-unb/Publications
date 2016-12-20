#include <iostream>
#include <fstream>
#include <sstream>
#include <cerrno>
#include <string>
#include <boost/program_options.hpp>
#include <TransmitterRobot.hpp>

void parseOpt(std::string *receiverTransmitter, std::string *remoteIP, int *remotePort, std::string *naoBrokerIP, int *naoBrokerPort, int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Print help mesages.")
        ("mode,m", po::value<std::string>(receiverTransmitter)->default_value("R"), "Robot Mode. T for Transmitter R for Receiver. Default: Receiver")
        ("rip", po::value<std::string>(remoteIP)->default_value("169.254.44.123"), "IP of the comm tester. Default: ")
        ("rport", po::value<int>(remotePort)->default_value(10042),"Port of the parent broker. Default: 10042")
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
        std::string parentBrokerIP;
        int parentBrokerPort;
        std::string receiverTransmitter;
        std::string remoteIP;
        int remotePort;

        setlocale(LC_NUMERIC, "C"); // Need this to for SOAP serialization of floats to work

        parseOpt(&receiverTransmitter, &remoteIP, &remotePort, &parentBrokerIP, &parentBrokerPort, argc, argv);

        if(std::toupper(receiverTransmitter[0]) == 'R')
        {
            //ReceiverRobot(remoteIP, remotePort);
        }
        else if(std::toupper(receiverTransmitter[0]) == 'T')
        {
            int type;
            TRobot robot;
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

                }

            }

        }


    return 0;
}
