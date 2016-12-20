#include "motionNAOqiFunction.hpp"
#include <cmath>

#define ELBOWROLLMAX 1.2

namespace MotionNAOqi
{

    void Stand()
	{
        // Create a proxy to ALLPosture and Motion
        AL::ALRobotPostureProxy Posture("127.0.0.1");
        AL::ALMotionProxy Motion("127.0.0.1");

        //Motion.setStiffnesses("Body", 1.0f);
        Posture.goToPosture("StandInit", 1.0);

	}

    void moveArm(float x, float y)
    {
        // Create a proxy to ALLPosture.
        AL::ALMotionProxy Motion("127.0.0.1");
        AL::ALValue shoulderRoll;
        AL::ALValue shoulderPitch;
        AL::ALValue shoulderRollAngle;

        //Motion.setStiffnesses("LArm", 1.0f);
        //Motion.setStiffnesses("RArm", 1.0f);
        //Motion.setStiffnesses("RShoulderRoll", AL::ALValue::array(1.0f, 1.0f));
        //Motion.setStiffnesses("RShoulderPitch", AL::ALValue::array(1.0f, 1.0f));


        shoulderRollAngle = atan2((y),(x+4500));

        if(y >= 0)
        {
            //esquerda
            shoulderRoll = "LShoulderRoll";
            shoulderPitch = "LShoulderPitch";


        }
        else if(y< 0)
        {
            //direita
            shoulderRoll = "RShoulderRoll";
            shoulderPitch = "RShoulderPitch";
        }

        Motion.setAngles(shoulderPitch, 0.0f , 0.5f);
        Motion.setAngles(shoulderRoll, shoulderRollAngle, 0.5f);

    }
};

