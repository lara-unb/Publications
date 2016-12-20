/*
 * Implement a (simple) example of the robot side of the no wifi challenge.
 * This file contains code for both the transmitter robot and communications
 * robot sides. The code uses blocking socket calls and very basic error
 * handling. It's main purpose is to demonstrate that the protocol
 * exchanges with the communications tester work correctly.
 *
 * When all is up and running the setup looks as follows:
 *
 *       transmitter robot, T ------[simulated no wifi]-----> receiver robot, R
 *              ^                                                      |
 *              |                                                      v
 *              +<----[ethernet]----- comms tester <----[ethernet]-----+
 *
 * For this simple example code it is important to do the steps in the following
 * order:
 * 1. start the comms tester is first. This will listen for incoming
 *    connections from R.
 * 2. Next start R which will connect to the comms tester and listen for
 *    connections from T.
 * 3. Then start T which connects to R and listens to connections from the
 *    comms tester.
 * 4. In the comms tester tool connect to T and once connected send
 *    location messages or data messages as required.
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <iostream>
#include <string>


#include "SPLNoWifiChallenge.h"
#include "TcpConnection.h"
#include "SimNoWifiPacket.h"



#define LOCATION_QUANT  32

#define CHECK_COUT_RETURN(cond,msg)   if (cond) { std::cout << (msg) << std::endl; return; }
#define CHECK_CERR_RETURN(cond,msg)      if (cond) { std::cerr << (msg) << std::endl; return; }

#define CHECK_CERR_RETURN_FALSE(cond,msg)      if (cond) { std::cerr << (msg) << std::endl; return false; }

// change these values to enable/disable different connections while
// testing the comms_robot tool itself
bool testingCommsTesterToTransmitterEnabled = true;
bool testingTransmitterToReceiverEnabled = true;
bool testingReceiverToCommsTesterEnabled = true;

// enabling this simulates a (very) slow communications non-wifi channel between robots
bool testingSimulateSlowNonWifi = true;

// additional offset between end of one fragment and start of next
// if 0, then fragments follow each other normally
// if negative then fragments will overlap
// if positive then fragments will leave gaps so that not all of original
// message is sent
// must be within the range of +/- (SIM_NO_WIFI_PAYLOAD_SIZE - sizeof(SimNoWifiDataPayloadHeader))
int testingFragmentPrevToNextOffset = 0;

// setting this to zero means no errors (obviously)
// setting to a value between 1 and 100 means that errors will be
// injected probabilistically into the data sent between transmitter
// and receiver robots
int testingPercentErrors = 0;


// ==========================================================================

void usageExit(int code) {
    std::cerr << "Usage:\n"
            "\trobotcomms T <listenPortForCommsTester> <receiverHost> <receiverPort>\n"
            "\tor\n"
            "\trobotcomms R <listenPortForTransmitter> <commsTesterHost> <commsTesterPort>\n\n";
    exit(code);
}

// ==========================================================================

long getTimeMs() {
    struct timespec ts;
    long ms;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    ms = (ts.tv_sec * 1000L) + (ts.tv_nsec / 1000000);

    return ms;
}

// ==========================================================================


void sendSimLocationMessage(TcpConnection& toReceiverRobot, int16_t x, int16_t y) {
    // test a location message

    std::cout << "Transmitter robot: send location to receiver\n";

    SimNoWifiPacket simNoWifiPacket;
    SimNoWifiLocation simNoWifiLocation;

    simNoWifiPacket.type = SIM_NO_WIFI_TYPE_LOCATION;

    // simulate a no wifi scheme which quantizes the x and y locations.
    // These are still valid provide they are in within 50 mm of
    // originally transmitted values
    simNoWifiLocation.x = (x / LOCATION_QUANT) * LOCATION_QUANT;
    simNoWifiLocation.y = (y / LOCATION_QUANT) * LOCATION_QUANT;

    std::cout << "  quantized x,y: " << x << ", " << y << " --> "
            << simNoWifiLocation.x << ", " << simNoWifiLocation.y << std::endl;

    memcpy(simNoWifiPacket.payload, &simNoWifiLocation,
            sizeof(simNoWifiLocation));

    int nSent = toReceiverRobot.sendData(&simNoWifiPacket,
            sizeof(simNoWifiPacket));
    CHECK_CERR_RETURN((nSent == -1),
            "Problem sending location to receiver robot");
}


void sendSimDataMessage(TcpConnection& toReceiverRobot, const uint8_t *data, int offsetIn, int lengthIn) {
    std::cout << "Transmitter robot: sending data payload to receiver (using sim non-wifi)\n";

    SimNoWifiPacket simNoWifiPacket;
    SimNoWifiDataPayloadHeader simNoWifiDataPayloadHeader;
    int remaining = lengthIn;
    int offset = offsetIn;

    simNoWifiPacket.type = SIM_NO_WIFI_TYPE_DATA;
    long startMs = getTimeMs();

    std::cout << "startTimeMs " << startMs << std::endl;

    while (remaining > 0) {
        int validlength = std::min(remaining,
                (int)(SIM_NO_WIFI_PAYLOAD_SIZE - sizeof(SimNoWifiDataPayloadHeader)));

        // first copy the data header
        simNoWifiDataPayloadHeader.length = validlength;
        simNoWifiDataPayloadHeader.offset = offset;
        memcpy(simNoWifiPacket.payload, &simNoWifiDataPayloadHeader,
                sizeof(SimNoWifiDataPayloadHeader));

        // then copy the actual data
        memcpy(simNoWifiPacket.payload + sizeof(simNoWifiDataPayloadHeader),
                data + offset, validlength);

        if (testingPercentErrors > 0) {
            uint8_t *data = simNoWifiPacket.payload + sizeof(simNoWifiDataPayloadHeader);
            for (int i = 0; i < validlength; i++)
                if ((rand() % 100) < testingPercentErrors)
                    data[i] += 1;
        }

        if (testingSimulateSlowNonWifi) {
            sleep(1);

            std::cout << "deltaTimeMs " << (getTimeMs() - startMs) << std::endl;
        }

        // check if we've exceeded the time budget for sending (with a
        // couple of seconds tolerance)
        if ((getTimeMs() - startMs) > (SPL_NO_WIFI_TIMEOUT_MS + 2000))
            return;


        int nSent = toReceiverRobot.sendData(&simNoWifiPacket, sizeof(simNoWifiPacket));
        CHECK_CERR_RETURN((nSent == -1), "Problem sending data to receiver robot");

        std::cout << "  sent sim non-wifi data: offset " << offset
                << ", length " << sizeof(simNoWifiPacket)
                << " (valid " << validlength << ")\n";

        // compensate for header info sent
        nSent -= sizeof(uint8_t) + sizeof(SimNoWifiDataPayloadHeader);

        // compensate for valid data
        nSent = std::min(nSent, validlength);

        // apply additional correction (for testing) to allow fragments
        // to overlap or leave gaps depending on the configured value.
        // Only do this if the complete data message has not been sent yet
        if ((remaining - nSent) > 0)
            nSent += testingFragmentPrevToNextOffset;

        offset += nSent;
        remaining -= nSent ;
    }
}


// ==========================================================================

void transmitterRobot(int listeningPort, const char* remoteIP, int remotePort)
{
    // we will allow the destructors for these objects to automatically
    // clean up the sockets if required

    TcpConnection fromCommsTester;
    TcpConnection toReceiverRobot;

    if (testingCommsTesterToTransmitterEnabled) {
        std::cout << "Transmitter robot: listen for incoming connections from comms tester on port"
                << listeningPort << std::endl;

        fromCommsTester.listenOn(listeningPort);
    }

    // in reality the communications with receiver robot will use the
    // custom non-Wifi (non-wired) method of communications your team has
    // developed

    if (testingTransmitterToReceiverEnabled) {
        std::cout << "Transmitter robot: connect to receiver robot (using simulated non-wifi) at "
                << remoteIP << " : " << remotePort << std::endl;

        toReceiverRobot.connectTo(remoteIP, remotePort);
    }

    // now wait for a connection from comms tester and then forward all messages
    // to receiver robot

    SPLNoWifiPacket packet;

    if (testingCommsTesterToTransmitterEnabled) {
        std::cout << "Transmitter robot: wait for incoming connection from comms tester\n";

        fromCommsTester.waitForIncomingConnection();

        std::cout << "Transmitter robot: connected to comms tester\n";

        // read message

        while (true) {
            int nRead = fromCommsTester.readData(&packet.header, sizeof(SPLNoWifiHeader));
            CHECK_COUT_RETURN((nRead <= 0), "End of data from comms tester");

            std::cout << "Transmitter robot: received header, type="
                    << (int) packet.header.type << std::endl;

            if (packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION) {
                // read the fixed length location message payload
                nRead = fromCommsTester.readData(&packet.payload.location,
                        sizeof(SPLNoWifiLocationPayload));
                CHECK_CERR_RETURN((nRead <= 0),
                        "End of data from comms tester after header");

                std::cout << "Transmitter robot: received location, x="
                        << packet.payload.location.x
                        << ", y=" << packet.payload.location.y
                        << std::endl;

                if (testingTransmitterToReceiverEnabled) {
                    sendSimLocationMessage(toReceiverRobot,
                            packet.payload.location.x,
                            packet.payload.location.y);
                }

            } else if (packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_DATA) {

                // read the offset and length of this fragment
                // NOTE: from the comms tester, the offset will always be 0
                nRead = fromCommsTester.readData(&packet.payload.data.header,
                        sizeof(SPLNoWifiDataPayloadHeader));
                CHECK_CERR_RETURN((nRead <= 0),
                        "End of data from comms tester after header");

                std::cout << "Transmitter robot: received dataPayload header, offset="
                        << packet.payload.data.header.fragmentOffset
                        << ", length=" << packet.payload.data.header.fragmentLength
                        << std::endl;

                nRead = fromCommsTester.readData(packet.payload.data.data,
                        packet.payload.data.header.fragmentLength);
                CHECK_CERR_RETURN((nRead <= 0), "End of data from comms tester after data payload header");

                std::cout << "Transmitter robot: received dataPayload data bytes, length="
                        << packet.payload.data.header.fragmentLength
                        << std::endl;

                if (testingTransmitterToReceiverEnabled) {
                    sendSimDataMessage(toReceiverRobot, packet.payload.data.data,
                            0, packet.payload.data.header.fragmentLength);
                }

            } else {
                std::cerr << "Unrecognised header type from comms tester\n";
                return; // rely on exit from function to clean up sockets
            }
        }
    } else {
        // no comms tester connection, so test transmitterToReceiver connection only

        if (!testingTransmitterToReceiverEnabled) {
            std::cerr << "commsTesterToTransmitter and transmitterToReceiver not enabled - what are you testing?\n";
            return;
        }

        sendSimLocationMessage(toReceiverRobot, -199, 3000);
        sendSimLocationMessage(toReceiverRobot, -2999, 75);

        const char *s = "This is a simple test data message to see if the data goes "
                "through OK.\nIt should be long enough to span multiple packets.";

        sendSimDataMessage(toReceiverRobot, (const uint8_t *) s, 0, strlen(s));
    }
}


// ==========================================================================

void receiverRobot(int listeningPort, const char* remoteIP, int remotePort)
{
    TcpConnection toCommsTester;
    TcpConnection fromTransmitterRobot;

    std::cout << "Receiver robot: listen for incoming connections from transmitter on port "
            << listeningPort << std::endl;

    fromTransmitterRobot.listenOn(listeningPort);

    // in reality the communications with receiver robot will use the
    // custom non-Wifi (non-wired) method of communications your team has
    // developed


    if (testingReceiverToCommsTesterEnabled) {
        std::cout << "Receiver robot: connect to comms tester at " << remoteIP
                << " : " << remotePort << std::endl;
        toCommsTester.connectTo(remoteIP, remotePort);
    }

    // now wait for a connection from transmitter and then forward all received
    // messages to comms tester

    std::cout << "Receiver robot: wait for incoming connection from transmitter\n";
    fromTransmitterRobot.waitForIncomingConnection();

    std::cout << "Receiver robot: transmitter connected\n";

    while (true) {
        SimNoWifiPacket simNoWifiPacket;

        int nRead = fromTransmitterRobot.readData(&simNoWifiPacket, sizeof(simNoWifiPacket));
        if (nRead == 0) {
            std::cout << "Normal end of data from transmitter\n";
            return;
        } else if (nRead < (int)sizeof(simNoWifiPacket)) {
            std::cerr << "failed to read complete packet from transmitter\n";
            return;
        }

        // received packet so print (and forward it on)
        std::cout << "Receiver robot: received simulated no-wifi packet, type "
                << (uint32_t) simNoWifiPacket.type << std::endl;


        if (simNoWifiPacket.type == SIM_NO_WIFI_TYPE_LOCATION) {
            // read from simNoWifiPacket payload into location and then forward it on

            SimNoWifiLocation simNoWifiLocation;

            memcpy(&simNoWifiLocation, simNoWifiPacket.payload,
                    sizeof(SimNoWifiLocation));

            std::cout << "  location: x=" << simNoWifiLocation.x
                    << ", y=" << simNoWifiLocation.y << std::endl;

            if (testingReceiverToCommsTesterEnabled) {
                std::cout << "forwarding...";

                SPLNoWifiPacket packet;

                packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION;
                packet.payload.location.x = simNoWifiLocation.x;
                packet.payload.location.y = simNoWifiLocation.y;

                int nSent = toCommsTester.sendData(&packet,
                        sizeof(SPLNoWifiHeader) + sizeof(SPLNoWifiLocationPayload));
                CHECK_CERR_RETURN((nSent == -1), "Problem sending location to comms tester");

                std::cout << "forwarded OK\n";
            }

        } else if (simNoWifiPacket.type == SIM_NO_WIFI_TYPE_DATA) {
            // read from simNoWifiPacket payload into data message and then forward it on

            // read the current data fragment
            SimNoWifiDataPayloadHeader simNoWifiDataPayloadHeader;

            memcpy(&simNoWifiDataPayloadHeader, simNoWifiPacket.payload,
                    sizeof(simNoWifiDataPayloadHeader));

            std::cout << "  data: offset=" << (int) simNoWifiDataPayloadHeader.offset
                    << ", length=" << (int) simNoWifiDataPayloadHeader.length << std::endl;

            if (testingReceiverToCommsTesterEnabled) {
                std::cout << "forwarding...";

                SPLNoWifiPacket packet;

                // now build the message for the comms tester

                // first fill in and copy the overall packet header
                packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_DATA;

                // now fill in and copy the data payload header

                packet.payload.data.header.fragmentOffset = simNoWifiDataPayloadHeader.offset;
                packet.payload.data.header.fragmentLength = simNoWifiDataPayloadHeader.length;

                // finally copy in the data payload itself
                memcpy(packet.payload.data.data,
                        simNoWifiPacket.payload
                                + sizeof(SimNoWifiDataPayloadHeader),
                        simNoWifiDataPayloadHeader.length);

                // now send the buffer to the comms tester
                int nSent = toCommsTester.sendData(&packet,
                        sizeof(SPLNoWifiHeader) + sizeof(SPLNoWifiDataPayloadHeader)
                                + simNoWifiDataPayloadHeader.length);
                CHECK_CERR_RETURN((nSent == -1), "Problem sending location to comms tester");

                std::cout << "forwarded OK\n";
            }

        } else {
            std::cerr << "Unrecognised header type from comms tester\n";
            return; // rely on exit from function to clean up sockets
        }
    }
}
