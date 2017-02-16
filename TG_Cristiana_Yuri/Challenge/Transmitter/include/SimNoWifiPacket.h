/*
 * @file: SimNoWifiPacket.h
 *
 * @author: Rudi Villing
 */

#ifndef SIMNOWIFIPACKET_H_
#define SIMNOWIFIPACKET_H_

// We'll simulate fixed length packets for non-wifi comms

#define SIM_NO_WIFI_PAYLOAD_SIZE 50

#define SIM_NO_WIFI_TYPE_LOCATION 1
#define SIM_NO_WIFI_TYPE_DATA 2

struct SimNoWifiPacket {
    uint8_t type;
    uint8_t payload[SIM_NO_WIFI_PAYLOAD_SIZE]; // this fixed size is always transmitted
                                          // - the contents depend on the type field
};

struct SimNoWifiDataPayloadHeader {
    uint16_t offset;
    uint8_t length;
};

struct SimNoWifiLocation {
    int16_t x;
    int16_t y;
};


#endif /* SIMNOWIFIPACKET_H_ */
