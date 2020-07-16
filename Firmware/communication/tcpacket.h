/*  *********************************************************************
    *  TouchRobotController
    *
    *  ODrive binary protocol                   File: tcpacket.h
    *
    *  This file is shared with the ODRive firmware to implement
    *  the touch robot binary protocol.
    *  
    *********************************************************************  
    *
    *  Author: Mitch Lichtenberg (mitch@lightmountain.org)
    *
    ********************************************************************* */

#ifndef _TCPACKET_H_
#define _TCPACKET_H_


//
// Commands
//

#define TCCMD_PING              0       // command codes sent to ODrive
#define TCCMD_POSITION          1       // set motor position
#define TCCMD_POSLIMITS         3       // set motor position with velocity feedforward
#define TCCMD_TRAPTRAJ          5       // use trapezoid trajectory to set position
#define TCCMD_ZEROENCODER       7       // zero the encoder (motors must be idle)
#define TCCMD_STATUS            9       // return robot status
#define TCCMD_GETIPROP          11      // Get an integer property by name
#define TCCMD_GETFPROP          13      // get a floating property by name
#define TCCMD_SETIPROP          15      // Set an integer property by name
#define TCCMD_SETFPROP          17      // Set a floating property by name
#define TCCMD_FEEDWATCHDOG      19      // feed the watchdog

#define TCCMD_RESPBIT           0x80    // Set if this is a response packet.


// 
// Status
//

#define TCSTS_OK                0       // Command successful
#define TCSTS_ERR_CMD           1       // Invalid command code
#define TCSTS_ERR_STATE         2       // Robot in incorrect state
#define TCSTS_ERR_PARAM         3       // Invalid parameter name/code
#define TCSTS_ERR_VALUE         4       // Parameter OK, but value invalid

//
// Command packets
//

#define TC_SEAL0        0xA1            // first 4 bytes are the seal
#define TC_SEAL1        0x55
#define TC_SEAL2        0xBB
#define TC_SEAL3        0xA2
#define TC_SEAL         0xA2BB55A1      // Little endian
#define TC_MAXLEN       128             // should never be longer than this

#define TC_HDRSIZE      12      // size of the "struct packet" below

typedef union {
    struct {
        uint32_t tc_seal;       // 00   Seal, 0xA155BBA2
        uint16_t tc_crc;        // 04   16-bit CRC of everything (asusming tc_crc held zero)
        uint8_t tc_cmd;         // 06   command
        uint8_t tc_seq;         // 07   sequence # (echoed in response)
        uint8_t tc_sts;         // 08   status (zero for commands)
        uint8_t tc_fparams;     // 09   # of floating point parameters
        uint8_t tc_iparams;     // 10   # of integer parameters
        uint8_t tc_sparams;     // 11   # of characters of string parameters (must include null)
        // rest of data         // 12   Header is 12 bytes
    } packet;
    uint8_t data[TC_MAXLEN];    // raw data
} tcpacket_t;

#define CRC16_POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/

static inline unsigned short crc16(char *data_p, unsigned short length)
{
    unsigned char i;
    unsigned int data;
    unsigned int crc = 0xffff;

    if (length == 0) {
        return (~crc);
    }

    do {
        for (i=0, data=(unsigned int)0xff & *data_p++;
             i < 8; 
             i++, data >>= 1) {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ CRC16_POLY;
            else  crc >>= 1;
        }
    } while (--length);

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);

    return (crc);
}

//
// Inline functions to put data into / get data from packets.
//

static inline void tcpkt_init(tcpacket_t *pkt)
{
    memset(pkt,0,sizeof(tcpacket_t));
    pkt->packet.tc_seal = TC_SEAL;
}

static inline size_t tcpkt_size(tcpacket_t *pkt)
{
    return TC_HDRSIZE + (pkt->packet.tc_fparams)*sizeof(float) + (pkt->packet.tc_iparams)*sizeof(int32_t) + (size_t)pkt->packet.tc_sparams;
}

static inline void tcpkt_addfloat(tcpacket_t *pkt, float v)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (pkt->packet.tc_fparams * sizeof(float));
    *((float *) ptr) = v;
    pkt->packet.tc_fparams++;
}

static inline void tcpkt_addint(tcpacket_t *pkt, int32_t v)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (pkt->packet.tc_fparams * sizeof(float)) + (pkt->packet.tc_iparams * sizeof(int32_t));
    *((int32_t *) ptr) = v;
    pkt->packet.tc_iparams++;
}

static inline void tcpkt_addstring(tcpacket_t *pkt, char *str)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (pkt->packet.tc_fparams * sizeof(float)) + (pkt->packet.tc_iparams * sizeof(int32_t));

    strcpy((char *) ptr, str);
    pkt->packet.tc_sparams = strlen(str)+1;
}

static inline float tcpkt_getfloat(tcpacket_t *pkt, int idx)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (idx * sizeof(float));
    return *((float *) ptr);
}

static inline int32_t tcpkt_getint(tcpacket_t *pkt, int idx)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (pkt->packet.tc_fparams * sizeof(float)) + (idx * sizeof(int32_t));
    return *((int32_t *) ptr);
}

static inline char *tcpkt_getstring(tcpacket_t *pkt)
{
    uint8_t *ptr;

    ptr = ((uint8_t *) pkt) + TC_HDRSIZE + (pkt->packet.tc_fparams * sizeof(float)) + (pkt->packet.tc_iparams * sizeof(int32_t));
    return (char *) ptr;
}

#endif
