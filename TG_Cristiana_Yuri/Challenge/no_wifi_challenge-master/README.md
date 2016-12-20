# RoboCup SPL No-WiFi Challenge CommsTester and Example Robot Code

This is test system that will be used during the RoboCup 2016 No-Wifi Challenge.

The set up to be used in the challenge is as follows:

        TransmitterRobot [T]  --> non wifi ---->  ReceiverRobot [R]
               ^                                         |
               |                                         |
          wired ethernet                          wired ethernet
               |                                         |
               |                                         v
               +<---------- CommsTester (on PC) <--------+

The CommsTester is provided as part of this package. The code for the
transmitter and receiver robots will be written by teams participating in the
challenge. Example robot code
which was used to test the CommsTester itself is also provided as part of
this package.

To set up a test using the supplied examples do the following:
  * First start CommsTester.jar on a PC (see below).
  * Next start the comms_robot on a Nao (which we will term the receiver robot) 
    in receiver mode (see below). 
    The receiver will exit immediately with an error if it cannot connect to the CommsTester.
  * Then start the comms_robot on a Nao (which we will term the transmitter robot) 
    in transmitter mode (see below).
    The transmitter will exit immediately if it cannot connect to the receiver robot.
  * Now, in the CommsTester, connect to transmitter.
  * At this point all participants should be connected and it is possible
    to send test Location messages or Data messages
  * Disconnect from the transmitter or simply close the CommsTester app
    to disconnect all participants and cause the transmitter and receiver
    robot code to exit.
    
# SPLNoWifiChallenge.h

This file defines the data structures and constants related to the
communications protocol used between the CommsTester and transmitter and
receiver robots.

It also defines the default port on which the CommsTester listens for
incoming connections from the receiver robot and the default
port on which the transmitter robot is expected to listen for connections from
the CommsTester.

# CommsTester

The CommsTester is located in the comms_tester directory.

For convenience an executable jar has already been built.
To build from source you need to use ant in the same directory as build.xml 

Note that the executable jar file depends
on another jar located in the libs directory so it is important to copy both
the CommsTester.jar and libs directory if moving it to another location.

When the CommsTester starts it immediately starts listening for incoming 
connections on the default CommsTester port defined in SPLNoWifiChallenge.h.
If you want to change this port, edit the listen port (right hand side of UI)
and then click the "reset connection R" button.

The default port for connecting to the transmitter robot is already filled
in (though you can change it to another value if you wish). The 
transmitter robot host name needs to be changed -- the default value of
localhost is not appropriate for general use.

The UI buttons are fairly self-explanatory.

All tests are logged and the CommsTester operator is expected to read the
log information to see whether a test passed or failed. (It should be
fairly clear.)

Output is logged to the embedded console in the app, to the terminal which
was used to start the jar file (if any), and to two log files in the logs 
subdirectory of the CommsTester. The oveview log file contains the same output as
appeared in the console window. The detail log file contains more detailed
information (primarily of use for debugging). The same log files are used
and appended to each time the CommsTester is run on the same day.

# comms_robot

The comms_robot directory contains example code which was used to test
the CommsTester itself and validate the communications protocol to be used
for the challenge.

Note: The example code using blocking socket calls so it is unlikely to be directly
usable as the basis for developing robot code. The error handling is also
simplistic.

The comms_robot code implements a simulated non-WiFi communications scheme 
which is implemented as a normal TCP connection with a different protocol
and packet size to the one defined in SPLNoWifiChallenge.h. There are a number
of variables at the top of the comms_robot.cpp file which may be configured
(by editing the file and rebuilding) to test the effect of errors in transmission,
duplicate or overlapping packets, slow connections, etc. 

For convenience the comms_robot executable has already been built (on
Ubuntu 14.04 64 bit and using the Aldebaran 2.1.3.3 cross tool chain).

To build from source, use qibuild to configure the project and then
switch to the created build directory and type make.

## Receiver mode

Copy the comms_robot executable to the Nao you will be using as the receiver.

To start the example code in receiver mode type

    comms_robot R <listenPortFromT> <commsTesterHost> <commsTesterPort>

where
    
 * `listenPort` is the port to listen for connections from the transmitter robot
 * `commsTesterHost` is the hostname of the CommsTester PC.
 * `commsTesterPort` should be set to the default CommsTester listen port
    unless you have changed it on the CommsTester and reset the connection to R.
    
NOTE: do not start the comms_robot receiver until after the CommsTester has
started. 

## Transmitter mode

Copy the comms_robot executable to the Nao you will be using as the transmitter.
To start the example code in transmitter mode type

    comms_robot T <listenPortfromCommsTester> <receiverHost> <receiverPort>

where
    
 * `listenPort` is the port to listen for connections from the CommsTester
 * `receiverHost` is the hostname of the receiver robot. (It can be localhost
    if you run the receiver and transmitter on the same robot).
 * `receiverPort` can be any value you wish but must match what is set when
    the receiver comms_robot is started. 
    
NOTE: do not start the comms_robot transmitter until after the 
comms_robot receiver has started. 
