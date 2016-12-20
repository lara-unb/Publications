#include <motionData.hpp>

//Overload of operator= to the 3 main structs


Command0& Command0::operator=(Command const& copy)
    {
        head    = copy.head;
        body    = copy.body;
        leds    = copy.leds;
        sonar   = copy.sonar;
        joints.shouldChange = copy.joints.shouldChange;
        joints.LShoulderPitch = copy.joints.LShoulderPitch;
        joints.RShoulderPitch = copy.joints.RShoulderPitch;
        joints.LShoulderRoll = copy.joints.LShoulderRoll;
        joints.RShoulderRoll = copy.joints.RShoulderRoll;
        return *this;
    }

Command0& Command0::operator=(Command1 const& copy)
{
    head    = copy.head;
    body    = copy.body;
    leds    = copy.leds;
    sonar   = copy.sonar;
    joints.shouldChange = copy.joints.shouldChange;
    joints.LShoulderPitch = copy.joints.LShoulderPitch;
    joints.RShoulderPitch = copy.joints.RShoulderPitch;
    joints.LShoulderRoll = copy.joints.LShoulderRoll;
    joints.RShoulderRoll = copy.joints.RShoulderRoll;
    return *this;
}

Command1& Command1::operator=(Command const& copy)
{
    head    = copy.head;
    body    = copy.body;
    leds    = copy.leds;
    sonar   = copy.sonar;
    joints.shouldChange = copy.joints.shouldChange;
    joints.LShoulderPitch = copy.joints.LShoulderPitch;
    joints.RShoulderPitch = copy.joints.RShoulderPitch;
    joints.LShoulderRoll = copy.joints.LShoulderRoll;
    joints.RShoulderRoll = copy.joints.RShoulderRoll;
    return *this;
}

Command1& Command1::operator=(Command0 const& copy)
{
    head    = copy.head;
    body    = copy.body;
    leds    = copy.leds;
    sonar   = copy.sonar;
    joints.shouldChange = copy.joints.shouldChange;
    joints.LShoulderPitch = copy.joints.LShoulderPitch;
    joints.RShoulderPitch = copy.joints.RShoulderPitch;
    joints.LShoulderRoll = copy.joints.LShoulderRoll;
    joints.RShoulderRoll = copy.joints.RShoulderRoll;
    return *this;
}

Command& Command::operator=(Command0 const& copy)
{
    head    = copy.head;
    body    = copy.body;
    leds    = copy.leds;
    sonar   = copy.sonar;
    joints.shouldChange = copy.joints.shouldChange;
    joints.LShoulderPitch = copy.joints.LShoulderPitch;
    joints.RShoulderPitch = copy.joints.RShoulderPitch;
    joints.LShoulderRoll = copy.joints.LShoulderRoll;
    joints.RShoulderRoll = copy.joints.RShoulderRoll;
    return *this;
}

Command& Command::operator=(Command1 const& copy)
{
    head    = copy.head;
    body    = copy.body;
    leds    = copy.leds;
    sonar   = copy.sonar;
    joints.shouldChange = copy.joints.shouldChange;
    joints.LShoulderPitch = copy.joints.LShoulderPitch;
    joints.RShoulderPitch = copy.joints.RShoulderPitch;
    joints.LShoulderRoll = copy.joints.LShoulderRoll;
    joints.RShoulderRoll = copy.joints.RShoulderRoll;
    return *this;
}
