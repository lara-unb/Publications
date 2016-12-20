/*
* Struct to save relevant motion information in shared memory.
*/
#pragma once

#include <stdint.h>

#include <iostream>

#include <../motion/runswift/types/ActionCommand.hpp>

/**
 * Command for controlling the body
 * Note: Some ActionType Commands WILL disable the head
 */
/*struct Body {
  // Predefined actions. These take precedence over walk parameters
  enum ActionType {
     NONE = 0,

     // Stand - common starting pose for all other actions
     STAND,   //1

     // Walk
     WALK,    //2
     DRIBBLE, //3

     // Actions
     GETUP_FRONT, GETUP_BACK,   //4,5
     KICK,                            //6
     INITIAL,                         //7
     DEAD, REF_PICKUP,                //8, 9
     OPEN_FEET,                       //10
     THROW_IN,                        //11
     GOALIE_SIT,                      //12
     GOALIE_DIVE_RIGHT,               //13
     GOALIE_DIVE_LEFT,                //14
     GOALIE_CENTRE,                   //15
     GOALIE_UNCENTRE,                 //16
     GOALIE_STAND,                    //17
     GOALIE_INITIAL,                  //18
     GOALIE_AFTERSIT_INITIAL,         //19
     DEFENDER_CENTRE,                 //20
     GOALIE_FAST_SIT,                 //21
     GOALIE_PICK_UP,                  //22
     MOTION_CALIBRATE,                //23
     STAND_STRAIGHT,                  //24
     LINE_UP,                         //25
     NUM_ACTION_TYPES
  };
  ActionType actionType;

  // Walk/Kick Parameters
  int forward; // How far forward (negative for backwards)  (mm)
  int left;  // How far to the left (negative for rightwards) (mm)
  float turn; // How much anti-clockwise turn (negative for clockwise) (rad)
  float power; // How much kick power (0.0-1.0)
  float bend;

  // Kick parameters
  float speed;
  float kickDirection;

  enum Foot {
     LEFT = 0,
     RIGHT
  };
  Foot foot;
  bool isFast;

  // Set this to true if you want the robot to do a kick that tries to kick it not straight
  // but angled, primarily to avoid an opponent straight ahead of it.
  bool misalignedKick;

  /**
   * Constructor for walks and kicks
   * @param at Action Type
   * @param f  How far forward (mm)
   * @param l  How far to the left (mm)
   * @param t  How much counter-clockwise turn (rad)
   * @param p  How much power
   * @param bend  Angle to bend knees (rad)
   * @param s  How much speed
   * @param k  Direction to kick (rad)
   * @param ft  Which foot to use
   * @param fast  go fast or not
   * @see http://runswift.cse.unsw.edu.au/confluence/display/rc2010/Movement%2C+walk%2C+kicks
   */
  /*Body(ActionType at, int f = 0, int l = 0, float t = 0.0, float p = 1.0,
       float bend = 15.0, float s = 1.0, float k = 0.0, Foot ft = LEFT, bool fast=false,
       bool misalignedKick=false)
     : actionType(at),
       forward(f),
       left(l),
       turn(t),
       power(p),
       bend(bend),
       speed(s),
       kickDirection(k),
       foot(ft),
       isFast(fast),
       misalignedKick(misalignedKick) {}

  /* Boost python makes using default arguements difficult.
   * Define an arguementless constructor to wrap
   */
  /*Body()
     : actionType(NONE),
       forward(0),
       left(0),
       turn(0),
       power(0),
       bend(0),
       speed(0),
       kickDirection(0),
       foot(LEFT),
       isFast(false),
       misalignedKick(false) {}

};*/

/*const uint8_t priorities[Body::NUM_ACTION_TYPES] = {
  0, // NONE
  0, // STAND
  0, // WALK
  0, // DRIBBLE
  2, // GETUP_FRONT
  2, // GETUP_BACK
  0, // KICK
  2, // INITIAL
  1, // DEAD
  0, // REF_PICKUP
  0, // OPEN_FEET
  0, // THROW_IN
  2, // GOALIE_SIT
  2, // GOALIE_DIVE_LEFT
  2, // GOALIE_DIVE_RIGHT
  2, // GOALIE_CENTRE
  2, // GOALIE_UNCENTRE
  0, // GOALIE_STAND
  0, // GOALIE_INITIAL
  0, // GOALIE_AFTERSIT_INITIAL
  2, // DEFENDER_CENTRE
  2, // GOALIE FAST SIT
  2,  // GOALIE_PICK_UP
  0, // MOTION_CALIBRATE
  0, // STAND_STRAIGHT
  0  // LINE_UP
};

/**
* Command for controlling the head
*/
/*struct Head {
  float yaw;      // LEFT-RIGHT motion. Positive is LEFT
  float pitch;    // UP-DOWN angle. Positive is DOWN
  bool isRelative; // TRUE to add to current head angles [DEFAULT]
  float yawSpeed; // Speed of the yaw [0.0, 1.0]
  float pitchSpeed; // Speed of the pitch [0.0, 1.0]

  /**
   * Constructor
   * @param y Yaw amount (Left is positive) (rad)
   * @param p Pitch amount (Down is positive) (rad)
   * @param r Enable relative adjustment (default). False for absolute
   * @param ys Yaw speed [0.0, 1.0]
   * @param ps Pitch speed [0.0, 1.0]
   */
/*  Head(float y, float p = 0.0, bool r = true,
       float ys = 1.0, float ps = 1.0) : yaw(y),
                                         pitch(p),
                                         isRelative(r),
                                         yawSpeed(ys),
                                         pitchSpeed(ps) {}

  Head()
     : yaw(0.0),
       pitch(0.0),
       isRelative(true),
       yawSpeed(1.0),
       pitchSpeed(1.0) {}

};

struct rgb {
  bool red;
  bool green;
  bool blue;

  rgb(bool r = false, bool g = false, bool b = false) : red(r),
                                                        green(g),
                                                        blue(b) {}

};

struct LED {

  uint16_t leftEar; // Number of left ear segments lit [10-bit field]
  uint16_t rightEar; // Number of right ear segments lit [10-bit field]
  rgb leftEye;     // Colour of left eye (default: white)
  rgb rightEye;    // Colour of right eye (default: white)
  rgb chestButton; // Colour of chest button (default: white)
  rgb leftFoot;    // Colour of left foot (default: off)
  rgb rightFoot;   // Colour of right foot (default: off)

  LED(rgb leye, rgb reye = rgb(true, true, true), rgb cb = rgb(true, true, true),
      rgb lf = rgb(), rgb rf = rgb()) : leftEar(0x3FF),
                                        rightEar(0x3FF),
                                        leftEye(leye),
                                        rightEye(reye),
                                        chestButton(cb),
                                        leftFoot(lf),
                                        rightFoot(rf) {}

  LED()
        : leftEar(0x3FF),
          rightEar(0x3FF),
          leftEye(rgb(true, true, true)),
          rightEye(rgb(true, true, true)),
          chestButton(rgb(true, true, true)),
          leftFoot(rgb()),
          rightFoot(rgb()) {}

};

/**
* Wrapper for the other action commands, makes it easier to pass them around
*/
struct armJoint
{
    bool shouldChange;
    std::vector<float> LShoulderPitch;
    std::vector<float> LShoulderRoll;
    std::vector<float> RShoulderPitch;
    std::vector<float> RShoulderRoll;
};

struct Command;
struct Command0;
struct Command1;

struct Command0 {
    ActionCommand::Head head;
    ActionCommand::Body body;
    ActionCommand::LED leds;
    float sonar;
    armJoint joints;


    Command0() : head(), body(ActionCommand::Body::NONE), leds(),
           sonar(-1.f) {}



    Command0(ActionCommand::Head h, ActionCommand::Body b, ActionCommand::LED l, float s) {
        head = h;
        body = b;
        leds = l;
        sonar = s;
    }

    Command0& operator=(Command const& copy);
    Command0& operator=(Command1 const& copy);
};

struct Command1 {
  ActionCommand::Head head;
  ActionCommand::Body body;
  ActionCommand::LED leds;
  float sonar;
  armJoint joints;

  Command1() : head(), body(ActionCommand::Body::NONE), leds(),
           sonar(-1.f) {}

  Command1(ActionCommand::Head h, ActionCommand::Body b, ActionCommand::LED l, float s) {
     head = h;
     body = b;
     leds = l;
     sonar = s;
  }

  Command1& operator=(Command const& copy);
  Command1& operator=(Command0 const& copy);
};

struct Command {
  ActionCommand::Head head;
  ActionCommand::Body body;
  ActionCommand::LED leds;
  float sonar;
  armJoint joints;


  Command() : head(), body(ActionCommand::Body::NONE), leds(),
           sonar(-1.f) {}

  Command(ActionCommand::Head h, ActionCommand::Body b, ActionCommand::LED l, float s) {
     head = h;
     body = b;
     leds = l;
     sonar = s;
  }

  Command& operator=(Command0 const& copy);
  Command& operator=(Command1 const& copy);
};

struct CommandIndex
{
    int index;

    CommandIndex() :
        index(0) {}
};

struct sonarRaw
{
    std::vector< std::vector <int> > sonarWindow;
};
