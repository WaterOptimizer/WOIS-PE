/******************************************************************************
 *                       Copyright (c) 2008, Jabil Circuit
 *
 * This source code and any compilation or derivative thereof is the sole
 * property of Jabil Circuit and is provided pursuant to a Software License
 * Agreement.  This code is the proprietary information of Jabil Circuit and
 * is confidential in nature.  Its use and dissemination by any party other
 * than Jabil Circuit is strictly limited by the confidential information
 * provisions of the Software License Agreement referenced above.
 *
 ******************************************************************************
 *
 * Project      : WaterOptimizer Irrigation System (WOIS)
 * Organization : WaterOptimizer, LLC
 * Module       : radio.h
 * Description  : This file defines the radio logic interfaces.
 *
 *****************************************************************************/

#ifndef __radio_H
#define __radio_H

/* MODULE radio */

#include "debug.h"
#include "global.h"
#include "extFlash.h"
#include "config.h"

//if you change this it must also be changed in file drvRadio.c function drvRadioAPIModeInit
//as well. it was a quick and dirty way to get it to work.
//line of code is similiar to commented out line below.
#define PAN_SYS_DEFAULT   0x0234
//uint8_t pATBufID[]={'A','T','I','D','0','2','3','4','\r'};

#define PAN_SYS_DEF_MSB   0
#define PAN_SYS_DEF_2MSB  2
#define PAN_SYS_DEF_3MSB  3
#define PAN_SYS_DEF_LSB   4
 
/******************************************************************************
 *
 *  IMPLEMENTATION CONSTANTS
 *
 *****************************************************************************/

/* Turn on support for XBee firmware version "ZB" */
#define RADIO_ZB

#define BULK_FW_HDR_SIZE        15      /* size of the header for the firmware file */
#define RADIO_MAXPACKET         100     /* Maximum packet buffer size */
#define RADIO_MAXPAYLOAD        72      /* Maximum packet payload data size */
#define RADIO_MAXAPPDATA        65      /* Maximum message application data */
#define RADIO_MAXSEGMENT        64      /* Maximum data xfer segment size */
#define RADIO_SZ_MAC_ID         8       /* Size of radio physical address */
#define RADIO_SZ_NET_AD         2       /* Size of radio network address */
#define RADIO_SZ_AT_CMD         2       /* Size of 2-char AT command name */

#define EXPANSION_CONNECTED     1       /* communication was received from an expansion unit */
#define EXPANSION_NOT_CONNECTED 0       /* communication was not received from an expansion unit */

/* Number of segments needed to transfer a configuration image */
#define RADIO_CFG_SEGS ((CONFIG_IMAGE_SIZE + RADIO_MAXSEGMENT - 1) / RADIO_MAXSEGMENT)

/* Number of segements needed to transfer a max flash firmware version */
#define RADIO_FW_SEGS  ((MAX_FW_IMAGE_SIZE + RADIO_MAXSEGMENT - 1) / RADIO_MAXSEGMENT)

/* Number of segments contained in the entire EEPROM */
#define RADIO_EEPROM_SEGS ((CONFIG_EEPROM_SIZE + RADIO_MAXSEGMENT - 1) / RADIO_MAXSEGMENT)

/* Number of segments needed to transfer 24hrs of Flow sensor data */
#define RADIO_FLOW_SEGS ((FLOW_DATA_SIZE + RADIO_MAXSEGMENT - 1) / RADIO_MAXSEGMENT)

/* Number of segments needed to transfer 24hrs of Level sensor data  */
#define RADIO_LEVEL_SEGS ((LEVEL_DATA_SIZE + RADIO_MAXSEGMENT - 1) / RADIO_MAXSEGMENT)

#define RADIO_OFFLINE_AI_SECS         7         /* AI cmd interval when Offline */
#define RADIO_INIT_FAIL_SECS          15        /* Init response time-out fail secs */
#define RADIO_EXPAN_FAIL_SECS         15        /* Expansion Bus time-out fail secs */
#define RADIO_EXPAN_STATUS_INTERVAL   45        /* number of seconds between checking radio comm with unit irrigating */

#ifdef DEBUG_TIMINGS_ENABLED
    #pragma message("DEBUG!!! RADIO_SNSCON_FAIL_SECS @ 120s")
    #define RADIO_SNSCON_FAIL_SECS        120
#else
    #define RADIO_SNSCON_FAIL_SECS        1200  /* Active Sensor Concentrator time-out fail secs */
#endif

#define RADIO_CMD_RETRY_SECS    5       /* Seconds between command retries */
#define RADIO_TXD_N_RETRIES     2       /* Number of Tx Data retries */
#define RADIO_TXD_RETRY_SECS    5       /* Seconds between Tx data retries */

/* Radio Status */
#define RADIO_STATUS_NOTPOP     0       /* Assume radio module not populated */
#define RADIO_STATUS_ONLINE     1       /* Radio is associated with a PAN */
#define RADIO_STATUS_OFFLINE    2       /* Radio not associated with a PAN */
#define RADIO_STATUS_SCANNING   3       /* Radio is scanning for PAN router */
#define RADIO_STATUS_FAILURE    4       /* Radio failure */
#define RADIO_STATUS_INIT       5       /* Radio initializing */

#define NUM_LOOPBACK_PKT        10     /* Number of loopback packets to send for each test */

/******************************************************************************
 *
 *  WATEROPTIMIZER RADIO PROTOCOL DEFINITIONS
 *
 *****************************************************************************/

#define RADIO_PROTOCOL_VER      0x01    /* WOIS Radio Protocol Version */

/* Maximum accumulator deltas; a delta value beyond max forces resync. */
#define RADIO_ET_MAX_DELTA      1000    /* ET Accumulator - Delta Max */
#define RADIO_RAIN_MAX_DELTA    1000    /* Rainfall Accumulator - Delta Max */

/* Force first Weather Update command to be used to resync accumulators. */
#define RADIO_ET_INIT          -1001    /* ET Accumulator Initial Value */
#define RADIO_RAIN_INIT        -1001    /* Rainfall Accumulator Initial Value */

/*
**  WOIS Message Types
*/
#define RADIO_TYPE_CMD          0x01    /* Command */
#define RADIO_TYPE_ACK          0x02    /* Command Acknowledgement */
#define RADIO_TYPE_XFER         0x03    /* Bulk Data Transfer */
#define RADIO_TYPE_LOOPBACK     0x1B    /* Loopback Test Reply */
#define RADIO_TYPE_SC_ASSOC     0x04    /* Sensor Concentrator Command */
#define RADIO_TYPE_SC_STATUS    0x05    /* Sensor Concentrator Command Acknowledgement */
#define RADIO_TYPE_SC_DEASSOC   0x06

/*
**  WOIS Message Data Sizes
*/
#define RADIO_CMD_HEADER_SIZE       7       /* Length of command header */
#define RADIO_ACK_HEADER_SIZE       7       /* Length of acknowledgement header */
#define RADIO_ACK_MIN_DATA_SIZE     10      /* Min acknowledgement data length */
#define RADIO_XFER_HEADER_SIZE      8       /* Bulk data transfer header size */
#define RADIO_SC_ASSOC_HEADER_SIZE  4       /* length of sensor concentrator associate header */
#define RADIO_SC_STATUS_HEADER_SIZE 27//19       /* length of sensor concentrator status header */

/*
**  WOIS Commands
*/
#define RADIO_CMD_NO_OP         0x00    /* No Op */
#define RADIO_CMD_INHIBIT_ON    0x01    /* Inhibit On */
#define RADIO_CMD_INHIBIT_OFF   0x02    /* Inhibit Off */
#define RADIO_CMD_FORCE_ON      0x03    /* Force On */
#define RADIO_CMD_ET_DATA       0x04    /* ET Data (DEPRECATED) */
#define RADIO_CMD_GET_MOIST     0x05    /* Get Moisture Values */
#define RADIO_CMD_GET_EXSTAT    0x06    /* Get Extended System Status */
#define RADIO_CMD_CFG_GET_SNAP  0x07    /* Save Config Snapshot for Upload */
#define RADIO_CMD_CFG_PUT_START 0x08    /* Start Config Image Download */
#define RADIO_CMD_CFG_PUT_APPLY 0x09    /* Apply Downloaded Config Image */
#define RADIO_CMD_GET_ET_VALUES 0x0A    /* Get ET Values (DEPRECATED) */

#define RADIO_CMD_GET_MANUFDATA 0x0B    /* Get Manufacture Data (FUTURE) */
#define RADIO_CMD_SET_ACTION    0x0C    /* Set Controller Action (FUTURE) */

#define RADIO_CMD_INIT_DATETIME 0x0D    /* Init Date/Time (DEBUG) */

#define RADIO_CMD_GET_RADIOSTAT 0x0E    /* Get Radio Status (FUTURE) */
#define RADIO_CMD_DIAGNOSE      0x0F    /* Diagnostic Command (FUTURE) */

#define RADIO_CMD_WEATHER_DATA  0x10    /* Weather Update */
#define RADIO_CMD_GET_MB_VALUES 0x11    /* Get Moisture Balance Values */
#define RADIO_CMD_SET_MB_VALUES 0x12    /* Set Moisture Balance Values */
#define RADIO_CMD_FW_PUT_START  0x13    /* Start Firmware Image Download */
#define RADIO_CMD_FW_PUT_APPLY  0x14    /* Apply Downloaded Firmware Image */
#define RADIO_CMD_IRR_START         0x15 /* Master telling expansion to start its irrigation */
#define RADIO_CMD_IRR_COMPLETE      0x16 /* expansion telling master pulse irrigation is complete */
#define RADIO_CMD_EXPANSION_STATUS  0x17 /* expansion unit relaying status information to master */
#define RADIO_CMD_IRR_STOP        0x18    /* command to unit to stop irrigation */
#define RADIO_CMD_IRR_RESUME      0x19    /* command to unit to resume irrigation */
#define RADIO_CMD_IRR_SKIP        0x1A    /* command to unit to skip irrigation */
#define RADIO_CMD_IRR_OFF         0x1B    /* command to unit to turn off irrigation */
#define RADIO_CMD_IRR_AUTO        0x1C    /* command to turn irrigation auto run on */
#define RADIO_CMD_IRR_STOP_OFF    0x1D    /* command to expansion to turn irrStop to off */
#define RADIO_CMD_GET_EXSTAT_2    0x1E    /* get extended system status part 2 runtimes per zones 0-23 */
#define RADIO_CMD_GET_EXSTAT_3    0x1F    /* get extended system status part 3 runtimes per zones 24-48 */ 
#define RADIO_CMD_DELETE_SC       0x20    /* tell expansion unit to delete SC */
#define RADIO_CMD_SC_IS_REMOVED   0x21    /* expansion telling master SC has been deleted */
#define RADIO_CMD_EXPAN_GET_CONFIG 0x22     /* expansion requesting configuration from the master unit */
#define RADIO_CMD_SEND_FLOW        0x26     /* Master telling expansion the flow meter value.


/* Engineering Debug/Experimental Commands */
#define RADIO_CMD_RC_TEST       0x90    /* Remote Control Test */
#define RADIO_CMD_RC_EVENT      0x91    /* Remote Control Event */
#define RADIO_CMD_RC_GETLCD     0x92    /* Remote Control Get LCD Data */
#define RADIO_CMD_EVENT_LOG     0x9E    /* Save Event Log */
#define RADIO_CMD_GET_FW_VER    0x9F    /* Get Firmware Version */

/*
**  WOIS Command Acknowledgements
*/
#define RADIO_ACK_NO_OP         0x00    /* No Op Ack */
#define RADIO_ACK_INHIBIT_ON    0x01    /* Inhibit On Ack */
#define RADIO_ACK_INHIBIT_OFF   0x02    /* Inhibit Off Ack */
#define RADIO_ACK_FORCE_ON      0x03    /* Force On Ack */
#define RADIO_ACK_ET_DATA       0x04    /* ET Data Ack (DEPRECATED) */
#define RADIO_ACK_ET_DATA_NW    0x84    /* ET Data Ack - Not in Weather Mode (DEPRECATED) */
#define RADIO_ACK_GET_MOIST     0x05    /* Get Moisture Values Ack */
#define RADIO_ACK_GET_EXSTAT    0x06    /* Get Extended System Status Ack */
#define RADIO_ACK_CFG_GET_SNAP  0x07    /* Save Config Snapshot Ack */
#define RADIO_ACK_CFG_PUT_START 0x08    /* Start Config Image Download Ack */
#define RADIO_ACK_CFG_PUT_APPLY 0x09    /* Apply Download Config Image Ack */
#define RADIO_ACK_GET_ET_VALUES 0x0A    /* Get ET Values Ack (DEPRECATED) */
#define RADIO_ACK_GET_MANUFDATA 0x0B    /* Get Manufacture Data Ack (FUTURE) */
#define RADIO_ACK_SET_ACTION    0x0C    /* Set Controller Action Ack (FUTURE) */
#define RADIO_ACK_INIT_DATETIME 0x0D    /* Init Date/Time Ack (DEBUG) */
#define RADIO_ACK_GET_RADIOSTAT 0x0E    /* Get Radio Status Ack (FUTURE) */
#define RADIO_ACK_DIAGNOSE      0x0F    /* Diagnostic Command Ack  (FUTURE)*/
#define RADIO_ACK_WEATHER_DATA  0x10    /* Weather Update Ack */
#define RADIO_ACK_GET_MB_VALUES 0x11    /* Get Moisture Balance Values Ack */
#define RADIO_ACK_SET_MB_VALUES   0x12    /* Set Moisture Balance Values Ack */
#define RADIO_ACK_FW_PUT_START    0x13    /* Start Firmware Image Download Ack */
#define RADIO_ACK_FW_PUT_APPLY    0x14    /* Apply Download Firmware Image Ack */
#define RADIO_ACK_FW_START_ERROR  0x15    /* Start Firmware Image Download Error */
#define RADIO_ACK_IRR_START     0x16    /* Master telling expansion to start its irrigation */
#define RADIO_ACK_IRR_COMPLETE  0x17    /* expansion telling master pulse irrigation is complete */
#define RADIO_ACK_EXPANSION_STATUS 0x18   /* expansion unit relaying status information to master */
#define RADIO_ACK_IRR_STOP        0x19    /* command to unit to stop irrigation */
#define RADIO_ACK_IRR_RESUME      0x1A    /* command to unit to resume irrigation */
#define RADIO_ACK_IRR_SKIP        0x1B    /* command to unit to skip irrigation */
#define RADIO_ACK_IRR_OFF         0x1C    /* command to unit to turn off irrigation */
#define RADIO_ACK_IRR_AUTO        0x1D    /* command to turn irrigation auto run on */
#define RADIO_ACK_IRR_STOP_OFF    0x1E    /* command to expansion to turn irrStop to off */
#define RADIO_ACK_GET_EXSTAT_2    0x1F    /* get extended system status part 2 runtimes per zones 0-23 */
#define RADIO_ACK_GET_EXSTAT_3    0x20    /* get extended system status part 2 runtimes per zones 24-48 */
#define RADIO_ACK_DELETE_SC       0x21    /* tell expansion unit to delete SC */
#define RADIO_ACK_SC_IS_REMOVED   0x22    /* expansion telling master SC has been deleted */
#define RADIO_ACK_EXPAN_GET_CONFIG 0x23     /* expansion requesting configuration from the master unit */
#define RADIO_ACK_SEND_FLOW       0x26
/* Engineering Debug/Experimental Command Acknowledgements */
#define RADIO_ACK_RC_TEST       0xA0    /* Radio Control Test Ack */
#define RADIO_ACK_RC_EVENT      0xA1    /* Radio Control Event Ack */
#define RADIO_ACK_RC_GETLCD     0xA2    /* Radio Control Get LCD Data Ack */
#define RADIO_ACK_EVENT_LOG     0xAE    /* Save Event Log Ack */
#define RADIO_ACK_GET_FW_VER    0xAF    /* Get Firmware Version */

/*
**  WOIS Bulk Data Transfer Mode - High Nibble
*/
#define RADIO_XMODE_CONFIG      0x10    /* WOIS Configuration Image */
#define RADIO_XMODE_WOIS_FW     0x20    /* WOIS Main Unit Firmware */
#define RADIO_XMODE_FLOW        0x30    /* 24hrs Flow Sensor Readings*/
#define RADIO_XMODE_LEVEL       0x40    /* 24hrs Level Sensor Readings */
#define RADIO_XMODE_EXP_FW      0x50    /* RFU: WOIS Expansion Unit Firmware */
#define RADIO_XMODE_RAD_FW      0x60    /* RFU: WOIS Radio Firmware */  
#define RADIO_XMODE_EEPROM      0xE0    /* EEPROM debug */

/*
**  WOIS Bulk Data Transfer Mode - Low Nibble
*/
#define RADIO_XMODE_GET_REQ     0x01    /* Get Data Request (from NOC) */
#define RADIO_XMODE_PUT_REQ     0x02    /* Put Data Request (from NOC) */
#define RADIO_XMODE_GET_ACK     0x03    /* Get Data Ack (from WOIS) */
#define RADIO_XMODE_GET_NACK    0x04    /* Get Data Nack (from WOIS) */
#define RADIO_XMODE_PUT_ACK     0x05    /* Put Data Ack (from WOIS) */
#define RADIO_XMODE_PUT_NACK    0x06    /* Put Data Nack (from WOIS) */

/*
**  WOIS Exception Flags
*/
#define RADIO_EXCEP_OFF         0x80    /* System is Turned Off */
#define RADIO_EXCEP_INHIBITED   0x40    /* Inhibit On mode */
#define RADIO_EXCEP_SYSERROR    0x20    /* System Error (e.g., no Date&Time) */
#define RADIO_EXCEP_HWFAULT     0x10    /* Hardware Fault (e.g., blown fuse) */
#define RADIO_EXCEP_PAUSED      0x08    /* Irrigation is Paused */
#define RADIO_EXCEP_RESERVED_2  0x04    /* Reserved for future use */
#define RADIO_EXCEP_RESERVED_1  0x02    /* Reserved for future use */
#define RADIO_EXCEP_RESERVED_0  0x01    /* Reserved for future use */

/*
**  WOIS System Status
*/
#define RADIO_SYSTAT_IDLE       0       /* Idle */
#define RADIO_SYSTAT_AUTORUN    1       /* Running Automatic Program */
#define RADIO_SYSTAT_TEST       2       /* Running Test */
#define RADIO_SYSTAT_MANUAL     3       /* Running Manual Start Program */
#define RADIO_SYSTAT_FORCE      4       /* Running Force On Program */

/* WOIS System Status high nibble flags for current mode of operation */
#define RADIO_SYSTAT_RUNTIME    0x20    /* Run Time Based Operation Mode */
#define RADIO_SYSTAT_SENSOR     0x40    /* Sensor Based Operation Mode */
#define RADIO_SYSTAT_WEATHER    0x60    /* Weather Based Operation Mode */
#define RADIO_SYSTAT_PULSE      0x80    /* Pulse Operation Mode */

/* Command Result Indication */
#define RADIO_RESULT_SUCCESS    1       /* Success */
#define RADIO_RESULT_FAILURE    0       /* Failure */

/* Weather Update Result Indication */
#define RADIO_WEATHER_MODE_ON   1       /* Weather Mode Enabled */
#define RADIO_WEATHER_MODE_OFF  0       /* Weather Mode Not Enabled */

/* Moisture Sensor Failure Codes (must be outside normal sensor value range) */
#define RADIO_SENSOR_NOT_CFG    0x80    /* Moisture Sensor Not Configured */
#define RADIO_SENSOR_FAILURE    0x81    /* Moisture Sensor Failure Detected */


/******************************************************************************
 *
 *  SENSOR CONCENTRATOR DEFINITIONS
 *
 *****************************************************************************/

#define SC_NUMSENSORS   4
#define SC_ASSOCIATE    1
#define SC_DEASSOCIATE  0

/******************************************************************************
 *
 *  XBEE RF MODULE API DEFINITIONS
 *
 *****************************************************************************/

#define RADIO_NET_ADDR_DFLT     0xFFFE  /* Default/Unknown Network Address */
#define RADIO_LB_PROFILE        0xC105  /* ZigBee Profile ID for Loopback */
#define RADIO_LB_ENDPOINT       0xE8    /* ZigBee Data Endpoint for Loopback */
#define RADIO_LB_CLUSTER        0x12    /* ZigBee Cluster ID for Loopback */

/*
**  XBee API Packet Types
*/
#define RADIO_API_COMMAND       0x08    /* AT Command */
#define RADIO_API_TXDATA        0x10    /* ZigBee Transmit Data */
#define RADIO_API_TXDATA_EX     0x11    /* ZigBee Explicit Transmit Data */
#define RADIO_API_REM_COMMAND   0x17    /* Remote AT Command */
#define RADIO_API_RESPONSE      0x88    /* AT Command Response */
#define RADIO_API_STATUS        0x8A    /* Modem Status */
#define RADIO_API_TXSTAT        0x8B    /* ZigBee Transmit Status */
#define RADIO_API_RXDATA        0x90    /* ZigBee Receive Data */
#define RADIO_API_EXPRXDATA     0x91    /* ZigBee Explicit Receive Data */
#define RADIO_API_NODE_IDENT    0x95    /* Node Identification Indicator NOT SUPPORT ON 900MHZ*/
#define RADIO_API_REM_RESPONSE  0x97    /* Remote AT Command Response NOT SUPPORT ON 900MHZ*/

/*
**  XBee API Modem Status Codes
*/
#define RADIO_MDM_HWRESET       0       /* Hardware Reset */
#define RADIO_MDM_WDRESET       1       /* WatchDog Reset */
#define RADIO_MDM_ASSOC         2       /* Associated */
#define RADIO_MDM_DISASSOC      3       /* Disassociated */
#define RADIO_MDM_LOSTSYNC      4       /* Synchronization Lost */
#define RADIO_MDM_COOREALIGN    5       /* Coordinator Realignment */
#define RADIO_MDM_COORESTART    6       /* Coordinator Started */

/*
**  XBee API Command Response Status Codes
*/
#define RADIO_RESP_OK           0       /* OK */
#define RADIO_RESP_ERROR        1       /* ERROR */
#define RADIO_RESP_BADCMD       2       /* Invalid Command */
#define RADIO_RESP_BADDATA      3       /* Invalid Data */

/*
**  XBee API ZigBee Tx Delivery Status Codes
*/
#define RADIO_TXSTAT_OK         0x00    /* Success */
#define RADIO_TXSTAT_CCAF       0x02    /* CCA Falure */
/* NOTE: Status of 0x14 has been seen in test, but is undefined by Digi. */
#define RADIO_TXSTAT_INVDEST    0x15    /* Invalid Destination Endpoint */
#define RADIO_TXSTAT_ACKFAIL    0x21    /* Network ACK Failure */
#define RADIO_TXSTAT_NOTJOINED  0x22    /* Not Joined to Network */
#define RADIO_TXSTAT_SELFADDR   0x23    /* Self-Addressed */
#define RADIO_TXSTAT_ADDRNFND   0x24    /* Address Not Found */
#define RADIO_TXSTAT_ROUTNFND   0x25    /* Route Not Found */

/*
**  XBee API ZigBee Tx Discovery Status Codes
*/
#define RADIO_TXDISC_NOVHD      0x00    /* No Discovery Overhead */
#define RADIO_TXDISC_ADDR       0x01    /* Address Discovery */
#define RADIO_TXDISC_ROUTE      0x02    /* Route Discovery */
#define RADIO_TXDISC_ADROUTE    0x03    /* Address and Route Discovery */

/*
**  Association Indication Status Codes
*/
#define RADIO_AISTAT_JOINED     0x00    /* Found and Joined with PAN */
#define RADIO_AISTAT_NOPANS     0x21    /* Scan found no PANs */
#define RADIO_AISTAT_NOVALPANS  0x22    /* Scan found no valid SC & ID PANs */
#define RADIO_AISTAT_NJEXP      0x23    /* PAN is not allowing Joining */
#define RADIO_AISTAT_NJFAIL     0x27    /* Node Joining attempt failed */
#define RADIO_AISTAT_COFAIL     0x2A    /* Coordiator Start attempt failed */
#define RADIO_AISTAT_CHECKING   0x2B    /* Checking for an existing coordinator */
#define RADIO_AISTAT_SCANNING   0xFF    /* Scanning for a Parent */

/*
**  D6 Codes
*/
#define RADIO_D6_RTS            1       /* Enable RTS signal pin */

/*
**  D7 Codes
*/
#define RADIO_D7_CTS            1       /* Enable CTS signal pin */

/*
**  JV Codes
*/
#define RADIO_JV_ENABLED        1       /* Enable Channel Verification */

/*
**  Sleep Mode Codes (Note: Only End Devices can sleep.)
*/
#define RADIO_SM_DISABLED       0       /* Sleep Disabled */
#define RADIO_SM_DTR            1       /* DTR signal pin sleep */
#define RADIO_SM_CYCLIC         4       /* Cyclic Sleep */



/******************************************************************************
 *
 *  FRAME IDENTIFIERS
 *
 *  NOTE:
 *      All frame identifiers use type uint8_t.  Frame identifiers are used
 *      in the XBee radio API for ZigBee transmit data frames and also for
 *      XBee radio AT commands.  Frame identifiers are returned by the API
 *      in ZigBee transmit status responses and XBee AT command responses and
 *      provide a means to match frames sent with the corresponding API result.
 *
 *      Frame identifiers are used by the WOIS radio logic subsystem in
 *      various ways.  The available frame identifier number space ranges from
 *      0x01 to 0xFF, with 0x00 reserved by XBee API to suppress an API result
 *      message.  Frame ID values with the high bit (0x80) set are reserved
 *      for specific AT Commands which have been instrumented in the subsystem
 *      logic.  Most of the remaining frame ID values are pooled for rotating
 *      use with ZigBee data transmit requests.  Frame ID values equal to the
 *      byte codes used by the API for Frame Delimiter and Escape (0x7E, 0x7D)
 *      are avoided for good measure (and possible efficiency considerations).
 *
 *****************************************************************************/

#define RADIO_CMD_USER              0x7A    /* Reserved for Win32 platform */

/*
**  ZIGBEE DATA TRANSMIT FRAME ID POOL
*/
#define RADIO_DATA_FRAME_ID_FIRST   0x01    /* First ZigBee Tx data frame id */
#define RADIO_DATA_FRAME_ID_MAX     0x79    /* Last ZigBee Tx data frame id */

/*
**  AT COMMAND FRAME IDENTIFIERS
*/
#define RADIO_CMD_AUTO      0x80    /* high bit set for all automated cmds */
#define RADIO_CMD_SH        0x81    /* Get MAC ID (Ser#) High */
#define RADIO_CMD_SL        0x82    /* Get MAC ID (Ser#) Low */
#define RADIO_CMD_AI        0x83    /* Get Association Indication */
#define RADIO_CMD_CH        0x84    /* Get Operating Channel */
#define RADIO_CMD_OP        0x85    /* Get Operating PAN ID */
#define RADIO_CMD_ID        0x86    /* Get Configured PAN ID */
#define RADIO_CMD_NI        0x87    /* Get Configured Node ID */
#define RADIO_CMD_VR        0x88    /* Get Radio Firmware Version */
#define RADIO_CMD_HV        0x89    /* Get Radio Hardware Version */
#define RADIO_CMD_WR        0x8A    /* Write to non-volatile memory */
#define RADIO_CMD_D6        0x8B    /* Get RTS flow control setting */
#define RADIO_CMD_D6_S      0x8C    /* Set RTS flow control enabled */
#define RADIO_CMD_P0_S      0x8D    /* Set P0 pin disabled */
#define RADIO_CMD_D0_S      0x8E    /* Set D0 pin disabled */
#define RADIO_CMD_ID_S      0x8F    /* Set Configured PAN ID */
#define RADIO_CMD_NI_S      0x90    /* Set Configured Node ID */
#define RADIO_CMD_RE        0x91    /* Restore Defaults (factory defaults) */
#define RADIO_CMD_JV_S      0x92    /* Set Channel Verification enabled */
#define RADIO_CMD_DB        0x93    /* Get Received Signal Strength */
#define RADIO_CMD_FR        0x94    /* Sowftare Reset on this Node */
#define RADIO_CMD_D7        0x95    /* Get CTS flow control setting */
#define RADIO_CMD_D7_S      0x96    /* Set CTS flow control enabled */
#define RADIO_CMD_ND        0x97    /* Node Discover. returned 9 seperate  
                                     * AT_CMD respone packets  for each node*/
#define RADIO_CMD_RC        0x98    /* Read the RSSI value of specified channel 0-11  */
//#define RADIO_CMD_NH_S      0x99    /* Set the number of hops for the radio */
//#define RADIO_CMD_CE_S      0xA0    /* Set CE */
//#define RADIO_CMD_CE        0xA1    /* get CE value */ 


/******************************************************************************
 *
 *  LOOPBACK TEST STATES
 *
 *****************************************************************************/

#define RADIO_LBS_IDLE      0x00    /* Loopback testing not in progress */
#define RADIO_LBS_START     0x01    /* Loopback test, start request */
#define RADIO_LBS_TESTING   0x02    /* Loopback test, waiting for response */
#define RADIO_LBS_SUCCESS   0x03    /* Loopback test, response received */
#define RADIO_LBS_FAIL      0x04    /* Loopback test, response timeout */


/******************************************************************************
 *
 *  EXPANSION BUS
 *
 *****************************************************************************/
#define RADIO_EXP_SEND_ALL    0x0   /* send expansion bus message to all units

/******************************************************************************
 *
 *  XBEE RF MODULE API PACKET STRUCTURES
 *
 *****************************************************************************/

/*
**  ZigBee Rx Data Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t phyAddr[RADIO_SZ_MAC_ID];   /* 64-bit Physical Address (MAC ID) */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t options;                    /* Options */
    uint8_t data[RADIO_MAXPAYLOAD];     /* Receive Payload Data */
} radioRxDataPacket_t;

/*
**  ZigBee Remote Command Response Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t phyAddr[RADIO_SZ_MAC_ID];   /* 64-bit Physical Address (MAC ID) */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t command[2];                 /* Name of Command */
    uint8_t status;                     /* status of message */
    uint8_t data[RADIO_MAXPAYLOAD];     /* Receive Payload Data */
} radioRxRemCmdPacket_t;

/*
**  ZigBee Tx Data Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t phyAddr[RADIO_SZ_MAC_ID];   /* 64-bit Physical Address MAC ID */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t bcastRadius;                /* Broadcast Radius */
    uint8_t options;                    /* Options */
    uint8_t data[RADIO_MAXPAYLOAD];     /* Transmit Payload Data */
} radioTxDataPacket_t;

/*
**  Remote Command Request Get Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t phyAddr[RADIO_SZ_MAC_ID];   /* 64-bit Physical Address MAC ID */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t options;                    /* Command Options */
    uint8_t atCommand[RADIO_SZ_AT_CMD]; /* AT Command name */
    uint8_t atParams[RADIO_MAXPAYLOAD];     /* Transmit Payload Data */
} radioRemoteCommandPacket_t;


/*
**  ZigBee Tx Data Packet using Explicit Addressing
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t phyAddr[RADIO_SZ_MAC_ID];   /* 64-bit Physical Address (MAC ID) */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t srcEndPt;                   /* Source Endpoint */
    uint8_t destEndPt;                  /* Destination Endpoint */
    uint16_t clusterId;                 /* Cluster ID */
    uint16_t profileId;                 /* Profile ID */
    uint8_t bcastRadius;                /* Broadcast Radius */
    uint8_t options;                    /* Options */
    uint8_t data[RADIO_MAXPAYLOAD];     /* Transmit Payload Data */
} radioTxDataExPacket_t;


/*
**  Modem Status Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t status;                     /* Modem Status */
} radioStatusPacket_t;


/*
**  ZigBee Tx Status Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t netAddr[RADIO_SZ_NET_AD];   /* 16-bit Network Address */
    uint8_t retryCount;                 /* Transmit Retry Count */
    uint8_t status;                     /* Transmit Status */
    uint8_t discovery;                  /* Discovery Status */
} radioTxStatPacket_t;


/*
**  AT Command Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t atCommand[RADIO_SZ_AT_CMD]; /* AT Command name */
    uint8_t atParams[RADIO_MAXPAYLOAD]; /* Optional AT Command Parameters */
} radioCommandPacket_t;


/*
**  AT Command Response Packet
*/
typedef struct
{
    uint8_t apiType;                    /* API packet type identifier */
    uint8_t frameId;                    /* Frame Identifier */
    uint8_t atCommand[RADIO_SZ_AT_CMD]; /* AT Command name */
    uint8_t status;                     /* Command Response Status */
    uint8_t data[RADIO_MAXPAYLOAD];     /* Command Response Data */
} radioResponsePacket_t;






/******************************************************************************
 *
 *  WATEROPTIMIZER RADIO PROTOCOL APPLICATION MESSAGE STRUCTURES
 *
 *****************************************************************************/

/*
**  WaterOptimizer Radio Protocol Message Header
*/
typedef struct
{
    uint8_t version;                    /* Protocol Version */
    uint8_t msgType;                    /* Message Type */
    uint8_t crcHigh;                    /* CRC-16 High Byte (MSB) */
    uint8_t crcLow;                     /* CRC-16 Low Byte (LSB) */
} radioMsgHeader_t;

/*
**  WaterOptimizer Command Message
*/
typedef struct
{
    radioMsgHeader_t hdr;               /* Message Header */
    uint8_t msgId;                      /* Command Message Identifier */
    uint8_t cmd;                        /* Command Code */
    uint8_t dataLen;                    /* Additional Command Data Length */
    uint8_t data[RADIO_MAXAPPDATA];     /* Additional Command Data */
} radioMsgCmd_t;

/*
**  WaterOptimizer Command Acknowledgement Message
*/
typedef struct
{
    radioMsgHeader_t hdr;               /* Message Header */
    uint8_t msgId;                      /* Command Message Identifier */
    uint8_t cmd;                        /* Command Acknowledgement Code */
    uint8_t dataLen;                    /* Additional Ack Data Length */
    uint8_t data[RADIO_MAXAPPDATA];     /* Additional Ack Data */
} radioMsgAck_t;

/*
**  WaterOptimizer Bulk Data Transfer Messages
*/
typedef struct
{
    radioMsgHeader_t hdr;               /* Message Header */
    uint8_t segHigh;                    /* Segment Number High Byte (MSB) */
    uint8_t segLow;                     /* Segment Number Low Byte (LSB) */
    uint8_t xferMode;                   /* Transfer Mode */
    uint8_t dataLen;                    /* Segment Data Length */
    uint8_t data[RADIO_MAXSEGMENT];     /* Segment Data */
} radioMsgXfer_t;


/*
**  WaterOptimizer Wireless Sensor Concentrator Command Message
*/
typedef struct
{
    radioMsgHeader_t hdr;               /* Message Header */
    uint8_t msgId;                      /* Command Message Identifier */
    uint8_t cmd;                        /* Command Code */
    uint8_t dataLen;                    /* Additional Command Data Length */
    uint8_t data[RADIO_MAXAPPDATA];     /* Additional Command Data */
} radioMsgSCCmd_t;

/*
**  WaterOptimizer Wireless Sensor Concentrator Command Acknowledgement Message
*/
typedef struct
{
    radioMsgHeader_t hdr;               /* Message Header */
    uint8_t msgId;                      /* Command Message Identifier */
    uint8_t cmd;                        /* Command Acknowledgement Code */
    uint8_t dataLen;                    /* Additional Ack Data Length */
    uint8_t data[RADIO_MAXAPPDATA];     /* Additional Ack Data */
} radioMsgSCAck_t;


/*
/** SC  transmit data packet
*/
typedef struct
{
    radioMsgHeader_t hdr;                   /* Message Header */
    int16_t temperature;                    /* Temperature in Celcius */
    uint16_t chargeRate;                    /* The solar cell average charge rate in J */
    uint16_t battVoltage;                   /* Battery voltage in % */
    uint16_t sensor_chan[SC_NUMSENSORS];    /* 4 Moisture sensors for now */
    uint8_t solenoidState;                  /* only using the 4 LS bits. 1 means ON and 0 means OFF */
    uint16_t rawADCSnsValue[SC_NUMSENSORS];  /* raw adc value that could be used for generic sensors in future */
}packetSCStatus_t;


/*
** SC ACK packet to Associate and Status Messages from SC
*/
typedef struct
{
   radioMsgHeader_t hdr;               /* Message Header */
   uint64_t controlMacId;              // MAC ID of this controller
   uint32_t sleepTime;                 // time for sleep in seconds
   uint8_t command;                    // command to tell SC to Assoc or Deassoc
   int16_t hibTemp;                     // temp at which the device goes to hibernation. in degrees C
   uint32_t hibThresTime;              /* time at which temp is above threshold 
                                          in order to come out of hibernation*/
   uint8_t solenoidState;               /* only using the 4 LS bits. 1 means ON and 0 means OFF */
}packetSCAckData_t;

/******************************************************************************
 *
 *  Radio Node Information Stucture
 *
 *****************************************************************************/

/*
**  Radio Node Info
*/
typedef struct
{
    uint8_t MY;                               /* API packet type identifier */
    uint8_t SH[RADIO_SZ_MAC_ID/2];            /* 32-bit Physical Address High Portion (MAC ID) */
    uint8_t SL[RADIO_SZ_MAC_ID/2];            /* 32-bit Physical Address Low Portion (MAC ID) */
    uint8_t NI[20];                           /* Node Identifier */
    uint8_t parent_NetAddr[RADIO_SZ_NET_AD];  /* Parent Network Address */
    uint8_t deviceType;                       /* Parent Network Address */
    uint8_t status;                           /* Parent Network Address */
    uint8_t profile[2];                       /* Parent Network Address */
    uint8_t mfrID[2];                         /* Parent Network Address */
} radioNodeInfo_t;



/******************************************************************************
 *
 *  Sensor Concentrator Status Information Stucture
 *
 *****************************************************************************/

/*
**  Sensor Concentrator Status Info
*/
typedef struct
{
    //int16_t temperature;                    /*Temperature in Celcius   */
    uint16_t chargeRate;                    /*The solar cell average charge rate in J  */
    uint16_t battVoltage;                    /*Battery voltage in mV  */
    //uint16_t sensor_chan[SC_NUMSENSORS];    /*4 Moisture sensors for now*/
    uint8_t solenoidState;                  /* only using the 4 LS bits. 1 means ON and 0 means OFF */
}sensorConMeasure_t;


/******************************************************************************
 *
 *  RADIO GLOBAL VARIABLES
 *
 *****************************************************************************/

extern uint8_t radioStatus;                 /* radio status */
extern uint8_t radioStatusCh;               /* last CH command response */
extern uint8_t radioStatusDb;               /* last received signal strength */
extern uint32_t radioStatusVr;              /* radio firmware version */
extern uint16_t radioStatusHv;              /* radio hardware version */
extern uint64_t radioMacId;                 /* radio physical addr (MAC ID) */
extern bool_t radioDebug;                   /* radio debug enable flag */
extern bool_t radioMonitorRss;              /* radio RSS monitor enable flag */
extern char radioLastMsgDesc[25];           /* last msg received description */
extern char radioLastMsgDate[21];           /* last msg received date/time */
extern uint8_t radioLbState;                /* loopback test state */
#ifdef RADIO_ZB
extern uint64_t radioStatusOp;              /* last OP command response */
#else
extern uint16_t radioStatusOp;              /* last OP command response */
#endif
extern bool_t radioCmdIrrStart;             /*denotes that was told to start early */
extern sensorConMeasure_t snsConInfo[];     /* structure to hold rx measurement values from sns con */
extern bool_t newSensorConcenFound;         /* denotes if there is a new sns con to deal with */
extern bool_t associateSCMode;              /* denotes if in associate mode */
extern uint64_t unassociatedSnsConMacId;    /* mac ID of sensor concentrator that is trying to associate */
extern int8_t irrSnsConSolUnitIndex;            /* index into the sensor concentrator list that needs to have solenoid state changed */
extern uint8_t irrSnsConSolenoidChan;           /* sensor concentrator solenoid channel to turn on. */
extern bool_t scCheckedIn;                      /* value denoting sc checked in so do put system back into pause */
extern bool_t scDeleteList[];                   /* list of SC index to de-associate, FALSE= dont delete, TRUE=delete */
extern uint8_t scDeleteIndex;                   /* SC index to delete */
extern uint32_t radioSnsConCheckinTime;         /* time ticks since sensor concentrator that is suppose to be
                                                 * turning irrigation ON or OFF has checked in last
                                                 */
extern uint8_t dtflag;
extern uint16_t dtid;
//extern uint8_t assocflag;
//extern uint8_t assocack;
//extern uint16_t statusflag;
//extern uint16_t statusack;
//extern uint8_t radioCE;

#ifdef WIN32
/* Global variable externs only for WIN32 platform */
extern uint32_t radioEtAccumulator;         /* ET Data Accumulator */
extern uint32_t radioRainAccumulator;       /* Rainfall Data Accumulator */
extern uint8_t radioTxDataRetriesRemaining; /* numbr of Tx data retries left */
extern radioTxDataPacket_t radioTxDataPkt;  /* last Tx data packet buffer */
extern uint16_t radioTxDataPktLen;          /* last Tx data packet length */
extern uint32_t radioTxDataTime;            /* last Tx data send tick count */
#endif

extern uint8_t expMoistValue[36];

/******************************************************************************
 *
 *  RADIO FUNCTION PROTOTYPES
 *
 *****************************************************************************/

/*
**  PUBLIC RADIO FUNCTIONS
*/
void radioInit(void);
void radioPoll(void);
void radioReset(void);
void radioResetHw(void);
void radioPanIdSet(void);
void radioConfigReset(void);
void radioLoopbackTestStart(void);
void radioLoopbackTestCancel(void);
void radioRemoveSensorAssocHandler(uint64_t sensorMAC);
void expansionBusSendCmd(uint8_t command,uint64_t macId);
uint8_t radioAddSensorAssocList(uint64_t sensorMAC);
uint8_t radioRemoveSensorAssocList(uint64_t sensorMAC);
/*
**  RADIO FUNCTIONS MADE PUBLIC ONLY FOR WIN32 PLATFORM
*/
uint8_t radioDataFrameId(void);
void radioMsgInsertCrc(const void *pMsg, uint8_t length);
void radioCommandInitSequence(void);
bool_t radioProtoCmdWeatherUpdate(uint32_t etAccum, uint32_t rainAccum);
bool_t radioCommandSend(uint8_t frameId,
                        const char *pCommand,
                        const uint8_t *pData,
                        int16_t lenData);
bool_t radioDataSend(uint8_t frameId,
                     uint16_t netAddr,
                     uint32_t phyAddrH,
                     uint32_t phyAddrL,
                     const uint8_t *pData,
                     int16_t lenData);
bool_t radioDataWrite(radioTxDataPacket_t *msg, uint16_t length);
bool_t radioRemoteCommandSend(uint8_t frameId,
                              uint16_t netAddr,
                              uint32_t phyAddrH,
                              uint32_t phyAddrL,
                              const char *pCommand,
                              const uint8_t *pData,
                              int16_t lenData);

/*
**  RADIO FUNCTIONS DEFINED ONLY FOR WIN32 PLATFORM
*/
#ifdef WIN32
void radioPacketRemAtResponse(const uint8_t *pBuf, int16_t length);
void radioTestProtocolXferHandler(const radioRxDataPacket_t *pPacket);
void RemoteControlTestAck(const char *pData, uint8_t lenData);
void RemoteControlEventAck(uint8_t msgIdent);
void RemoteControlGetLcdAck(const char *pData, uint8_t lenData);
void CloneStartAck(void);
void CloneApplyAck(uint8_t status, uint16_t offset);
void CloneUpStartAck(void);
void CloneEvtStartAck(const char *pData, uint8_t lenData);
void GetMoistAck(const char *pData, uint8_t lenData);
void GetExStatAck(const char *pData, uint8_t lenData);
void GetMbValuesAck(const char *pData, uint8_t lenData);
void SetMbValuesAck(const char *pData, uint8_t lenData);
void WeatherUpdateAck(const char *pData, uint8_t lenData);
void InitDateTimeAck(const char *pData, uint8_t lenData);
void GetFwVerAck(const char *pData, uint8_t lenData);
#endif


/* END radio */

#endif
