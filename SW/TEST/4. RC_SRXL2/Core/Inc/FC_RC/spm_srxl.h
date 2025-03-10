/*
MIT License

Copyright (c) 2019-2023 Horizon Hobby, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

https://github.com/SpektrumRC/SRXL2
*/


#ifndef INC_FC_RC_SPM_SRXL_H_
#define INC_FC_RC_SPM_SRXL_H_

#ifdef __cplusplus
extern "C"
{
#endif



//      7.1 General Overview


//      7.2 Handshake Packet
#define SRXL_HANDSHAKE_ID       (0x21)

// Supported additional baud rates besides default 115200
// NOTE: Treated as bitmask, ANDed with baud rates from slaves
#define SRXL_BAUD_115200        (0x00)
#define SRXL_BAUD_400000        (0x01)
//#define SRXL_BAUD_NEXT_RATE   (0x02)
//#define SRXL_BAUD_ANOTHER     (0x04)

// Bit masks for Device Info byte sent via Handshake
#define SRXL_DEVINFO_NO_RF              (0x00)  // This is the base for non-RF devices
#define SRXL_DEVINFO_TELEM_TX_ENABLED   (0x01)  // This bit is set if the device is actively configured to transmit telemetry over RF
#define SRXL_DEVINFO_TELEM_FULL_RANGE   (0x02)  // This bit is set if the device can send full-range telemetry over RF
#define SRXL_DEVINFO_FWD_PROG_SUPPORT   (0x04)  // This bit is set if the device supports Forward Programming via RF or SRXL


//      7.3 Bind Info Packet
#define SRXL_BIND_ID                    (0x41)
#define SRXL_BIND_REQ_ENTER             (0xEB)
#define SRXL_BIND_REQ_STATUS            (0xB5)
#define SRXL_BIND_REQ_BOUND_DATA        (0xDB)
#define SRXL_BIND_REQ_SET_BIND          (0x5B)

// Bit masks for Options byte
#define SRXL_BIND_OPT_NONE              (0x00)
#define SRXL_BIND_OPT_TELEM_TX_ENABLE   (0x01)  // Set if this device should be enabled as the current telemetry device to tx over RF
#define SRXL_BIND_OPT_BIND_TX_ENABLE    (0x02)  // Set if this device should reply to a bind request with a Discover packet over RF
#define SRXL_BIND_OPT_US_POWER          (0x04)  // Set if this device should request US transmit power levels instead of EU

// Current Bind Status
typedef enum
{
    NOT_BOUND           = 0x00,
    // Air types
    DSM2_1024_22MS      = 0x01,
    DSM2_1024_MC24      = 0x02,
    DSM2_2048_11MS      = 0x12,
    DSMX_22MS           = 0xA2,
    DSMX_11MS           = 0xB2,
    // Surface types (corresponding Air type bitwise OR'd with 0x40)
    SURFACE_DSM1        = 0x40,
    SURFACE_DSM2_16p5MS = 0x63,
    DSMR_11MS_22MS      = 0xE2,
    DSMR_5p5MS          = 0xE4,
} BIND_STATUS;


//      7.4 Parameter Configuration
#define SRXL_PARAM_ID           (0x50)
#define SRXL_PARAM_REQ_QUERY    (0x50)
#define SRXL_PARAM_REQ_WRITE    (0x57)


//      7.5 Signal Quality Packet
#define SRXL_RSSI_ID            (0x55)
#define SRXL_RSSI_REQ_REQUEST   (0x52)
#define SRXL_RSSI_REQ_SEND      (0x53)


//      7.6 Telemetry Sensor Data Packet
#define SRXL_TELEM_ID           (0x80)


//      7.7 Control Data Packet
#define SRXL_CTRL_ID                (0xCD)
#define SRXL_CTRL_BASE_LENGTH       (3 + 2 + 2) // header + cmd/replyID + crc
#define SRXL_CTRL_CMD_CHANNEL       (0x00)
#define SRXL_CTRL_CMD_CHANNEL_FS    (0x01)
#define SRXL_CTRL_CMD_VTX           (0x02)
#define SRXL_CTRL_CMD_FWDPGM        (0x03)



#endif /* INC_FC_RC_SPM_SRXL_H_ */
