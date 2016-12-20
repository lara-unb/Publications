/*
* Struct to save relevant kinematics information in shared memory.
*/
#pragma once

#include <stdint.h>

#include <iostream>

struct KinematicsParameters
{
    float cameraPitchTop;
    float cameraYawTop;
    float cameraRollTop;

    float cameraYawBottom;
    float cameraPitchBottom;
    float cameraRollBottom;

    float bodyPitch;
    
    KinematicsParameters() :
        cameraYawBottom(1.4),
        cameraPitchBottom(2.75),
        cameraRollBottom(0.05),
        cameraYawTop(1.4),
        cameraPitchTop(2.4),
        cameraRollTop(1.3),
        bodyPitch(3.3)
        {}

};



/*//  These classes support stream output for debugging

static inline std::ostream & operator<<(std::ostream &out, const KinematicsParameters &a) {
    out << '{ cPT' << a.cameraPitchTop << ", cYT" << a.cameraYawTop << ", cRT" << a.cameraRollTop << ", cYB" << a.cameraYawBottom <<  ", cPB" << a.cameraPitchBottom <<  ", cRB" << a.cameraRollBottom <<  ", bP" << a.bodyPitch << '}';
  return out;
}
*/

