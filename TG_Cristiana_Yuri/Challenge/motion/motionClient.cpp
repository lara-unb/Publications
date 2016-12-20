#include <iostream>
#include <fstream>
#include <sstream>
#include <cerrno>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alvalue/alvalue.h>

#include <unBoard.hpp>
#include <motionData.hpp>
#include <kinematicsData.hpp>
#include <ButtonData.hpp>
#include <Odometry.hpp>
#include <perception/kinematics/SonarFilter.hpp>
#include <types/SensorValues.hpp>
#include <types/ActionCommand.hpp>

//#include <runswift/types/ActionCommand.hpp>


#define MIN_STANDING_WEIGHT 0.55f
#define FALLEN 9
#define FALLING 8
#define FALLEN_ANG 70
#define FALLING_ANG 45

float filtered_fsr_sum = 0;

//######################### GLOBALS -> CLASS VARIABLES WHEN COPIED TO BEHAVIOR ###################################
unBoard<Command0> CommandBoard0;
unBoard<Command1> CommandBoard1;
unBoard<CommandIndex> indexBoard;
unBoard<ButtonData> buttonBoard;

//################################################################################################################

ActionCommand::Body::ActionType getUpIfFalling(SensorValues &s) {
    float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft]
                    + s.sensors[Sensors::LFoot_FSR_FrontRight]
                    + s.sensors[Sensors::LFoot_FSR_RearLeft]
                    + s.sensors[Sensors::LFoot_FSR_RearRight]
                    + s.sensors[Sensors::RFoot_FSR_FrontLeft]
                    + s.sensors[Sensors::RFoot_FSR_FrontRight]
                    + s.sensors[Sensors::RFoot_FSR_RearLeft]
                    + s.sensors[Sensors::RFoot_FSR_RearRight];
    filtered_fsr_sum = filtered_fsr_sum + 0.2 * (fsr_sum - filtered_fsr_sum);
    // Verificar se o robo sera REF_PICKUP
    // Precisa de uma variavel externa filtered_frs_sum para armazenar o valor

    float ang[2] = {RAD2DEG(s.sensors[Sensors::InertialSensor_AngleX]),
                   RAD2DEG(s.sensors[Sensors::InertialSensor_AngleY])};
    std::cout << "X: " << ang[0] << " Y: " << ang[1] << std::endl;
    if(ang[1] < -FALLEN_ANG || ang[0] < -FALLEN_ANG){
        return ActionCommand::Body::GETUP_BACK;
    } else if(ang[1] > FALLEN_ANG || ang[0] > FALLEN_ANG){
        return ActionCommand::Body::GETUP_FRONT;
    } else if (ABS(ang[0]) > FALLING_ANG || ABS(ang[1]) > FALLING_ANG) {
        return ActionCommand::Body::DEAD;
    } else if (filtered_fsr_sum < MIN_STANDING_WEIGHT) {
        return ActionCommand::Body::REF_PICKUP;
    }
    else return ActionCommand::Body::NONE;
}

ActionCommand::Body Walk (const float& x, const float& y, const float& theta)
{
    ActionCommand::Body body;

    body.actionType = ActionCommand::Body::WALK;
    body.forward = x;
    body.left = y;
    body.turn = theta;
    body.power = 1.0;
    body.bend = 15.0;

    return body;
}

ActionCommand::Body Kick (const float& x, const float& y, const float& direction = 0.0, const ActionCommand::Body::Foot& foot = ActionCommand::Body::LEFT, const bool& misaligned = false)
{
    ActionCommand::Body body;

    body.actionType = ActionCommand::Body::KICK;
    body.ballX = x;
    body.ballY = y;
    body.kickDirection = direction;
    body.foot = foot;
    body.misalignedKick = misaligned;
    body.power = 1.0;
    body.speed = 1.0;
    body.bend = 15.0;

    return body;
}

ActionCommand::Head MoveHead (const float& pitch, const float& yaw, const float& pSpeed = 1.0, const float& ySpeed = 1.0, const bool& isRelative = true)
{
    ActionCommand::Head head;

    head.pitch = pitch;
    head.yaw = yaw;
    head.pitchSpeed = pSpeed;
    head.yawSpeed = ySpeed;
    head.isRelative = isRelative;

    return head;
}

ActionCommand::Body Stop()
{
    ActionCommand::Body body;

    body.actionType = ActionCommand::Body::WALK;
    body.forward = 0.0;
    body.left = 0.0;
    body.turn = 0.0;
    body.power = 1.0;
    body.bend = 15.0;

    return body;
}

ActionCommand::Body Stand()
{
    ActionCommand::Body body;

    body.actionType = ActionCommand::Body::STAND;
    body.forward = 0.0;
    body.left = 0.0;
    body.turn = 0.0;
    body.power = 1.0;
    body.bend = 15.0;

    return body;
}

ActionCommand::Body Penalize()
{
    ActionCommand::Body body;

    body.actionType = ActionCommand::Body::REF_PICKUP;
    body.forward = 0.0;
    body.left = 0.0;
    body.turn = 0.0;
    body.power = 1.0;
    body.bend = 15.0;

    return body;
}

void SendCommand(const Command& command)
{
    CommandIndex index = indexBoard.load();
    index.index = (index.index + 1) % 2;
    if(index.index)
    {
        Command1 command1;
        command1 = command;
        CommandBoard1.save(command1);
        indexBoard.save(index);
    }
    else
    {
        Command0 command0;
        command0 = command;
        CommandBoard0.save(command0);
        indexBoard.save(index);
    }
}

bool ButtonPressed()
{
    ButtonData buttons;
    buttons = buttonBoard.load();
    if(buttons.button.pop(1))
    {
        //Chest Button pressed once
        return true;
    }
    return false;
}


void parseOpt(std::string *naoBrokerIP, int *naoBrokerPort, int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Print help mesages.")
        ("pip", po::value<std::string>(naoBrokerIP)->default_value("127.0.0.1"), "IP of the parent broker. Default: NAOMAR.local")
        ("pport", po::value<int>(naoBrokerPort)->default_value(9559),"Port of the parent broker. Default: 9559");
    po::positional_options_description positionalOptions;
    po::variables_map vm; // Map containing all the options with their values
    try
    {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

        if (vm.count("help"))
        {
            std::cout << "Motion client module."                                   << std::endl;
            std::cout << "Can be executed locally or remotelly."                   << std::endl;
            std::cout                                                              << std::endl;
            std::cout << "Usage:"                                                  << std::endl;
            std::cout << "  " << argv[0] << " [options] [input] "                  << std::endl;
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

int main(int argc, char* argv[])
{
    std::string parentBrokerIP;
    int parentBrokerPort;
    
    setlocale(LC_NUMERIC, "C"); // Need this to for SOAP serialization of floats to work

    parseOpt(&parentBrokerIP, &parentBrokerPort, argc, argv);
    
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

    Command0 command0;
    Command command;
    CommandIndex index;

    std::cout << "Debug sonar UnBoard" << std::endl;
    unBoard<sonarRaw> sonarRawBoard;
    std::cout << "Debug odometry UnBoard" << std::endl;
    unBoard<Odometry> odometryBoard;
    std::cout << "Debug sensor UnBoard" << std::endl;
    unBoard<SensorValues> sensorBoard;

    std::cout << "Board Created" << std::endl;
    SensorValues tmp;
    sensorBoard.save(tmp);

    index.index = 0;

    command0.body = Stand();
    CommandBoard0.save(command0);
    indexBoard.save(index);

    std::cout << "Board initialized" << std::endl;

    SonarFilter sonarFilter;

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

    bool flagStop = false;

    std::string input;
    while(!flagStop)
    {
        /*index = indexBoard.load();
        index.index = (index.index + 1) % 2;
        if(index.index)
        {
            command1.body.actionType = ActionCommand::Body::NONE;
            command1.body.forward = 0;
            //command1.head.yaw = 0.6f;
            //command1.head.yawSpeed = 0.5f;
            command1.leds.leftEye = 0xAAA;
            CommandBoard1.save(command1);
            indexBoard.save(index);
        }
        else
        {
            command0.body.actionType = ActionCommand::Body::NONE;
            command0.body.forward = 0;
            //command0.head.yaw = 0.6f;
            //command0.head.yawSpeed = 0.5f;
            command0.leds.leftEye = 0xAAA;
            CommandBoard0.save(command0);
            indexBoard.save(index);
        }*/

        SensorValues sensorValues = sensorBoard.load();
        std::cout << "S: " << sensorValues.sensors[0] << std::endl;
        if(sensorValues.sensors[0] != NAN)
        {
            ActionCommand::Body::ActionType fallingState = getUpIfFalling(sensorValues);
            if(fallingState != ActionCommand::Body::NONE)
            {
                command.body.actionType = fallingState;
                command.body.power = 1.0;
                command.body.bend = 15.0;
                command.body.forward = 0.0;
                command.body.left = 0.0;
                command.body.turn = 0.0;
                SendCommand(command);
                continue;
            }
        }
        std::cout << "Input: (S = stand, F = Forward, B = Backward, L = Left, R = Right, G = Get up front, O = stop, P = print sonar and odometry, q = end)" << std::endl;
        std::cin >> input;

        if(input == "S" || input == "s")
        {
            command.body = Stand();
            SendCommand(command);
        }
        else if(input == "F" || input == "f")
        {
            command.body = Walk(300.0, 0.0, 0.0);
            SendCommand(command);
        }
        else if(input == "B" || input == "b")
        {
            command.body = Walk(-300.0, 0.0, 0.0);
            SendCommand(command);
        }
        else if(input == "L" || input == "l")
        {
            command.body = Walk(0.0, 300.0, 0.0);
            SendCommand(command);
        }
        else if(input == "R" || input == "r")
        {
            command.body = Walk(0.0, -300.0, 0.0);
            SendCommand(command);
        }
        else if(input == "G" || input == "g")
        {

        }
        else if(input == "O" || input == "o")
        {
            command.body = Stop();
            SendCommand(command);
        }
        else if(input == "P" || input == "p")
        {
            //Sonar
            sonarRaw sonarRawData = sonarRawBoard.load();
            if(!sonarRawData.sonarWindow.empty())
            {
                sonarFilter.update(sonarRawData.sonarWindow);

                std::cout << "Sonar L: " << sonarFilter.sonarFiltered[Sonar::LEFT][0] << " M: " << sonarFilter.sonarFiltered[Sonar::MIDDLE][0] << " R: " << sonarFilter.sonarFiltered[Sonar::RIGHT] << std::endl;
            }
            //Odometry
            /*Odometry odometry = odometryBoard.load();

            std::cout << "Odometry Forward: " << odometry.forward << " Left: " << odometry.left << " Turn: " << odometry.turn << std::endl;*/
            command.body = Penalize();
            SendCommand(command);

        }
        else if(input == "K" || input == "k")
        {
            command.body = Kick(160.0, 45.0, 0.0);
            SendCommand(command);
        }
        else if(input == "Q" || input == "q")
        {
            flagStop = true;
        }
        else
        {
            command.body.actionType = ActionCommand::Body::NONE;
            command.body.forward = 0;
            command.body.left = 0;
            SendCommand(command);
        }
    }

    try
    {
        Motion->callVoid("StopLoop");
    }
    catch (const AL::ALError& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
