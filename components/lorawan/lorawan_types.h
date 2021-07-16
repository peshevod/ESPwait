/*
 * lorawan_types.h
 *
 *  Created on: 16 июл. 2021 г.
 *      Author: ilya_000
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "eui.h"

#ifndef COMPONENTS_LORAWAN_LORAWAN_TYPES_H_
#define COMPONENTS_LORAWAN_LORAWAN_TYPES_H_

#define INVALID_VALUE                           0xFF

#define ENABLED                                 1
#define DISABLED                                0

#define ACCEPTED                                1
#define REJECTED                                0

#define RESPONSE_OK                             1
#define RESPONSE_NOT_OK                         0

#define FLAG_ERROR                              0
#define FLAG_OK                                 1

#define MCAST_ENABLED                           1
#define MCAST_DISABLED                          0

#define RESERVED_FOR_FUTURE_USE                 0

#define MAXIMUM_BUFFER_LENGTH                   271

#define CLASS_C_RX_WINDOW_SIZE                  0

//Data rate (DR) encoding
#define DR0                                     0
#define DR1                                     1
#define DR2                                     2
#define DR3                                     3
#define DR4                                     4
#define DR5                                     5
#define DR6                                     6
#define DR7                                     7
#define DR8                                     8
#define DR9                                     9
#define DR10                                    10
#define DR11                                    11
#define DR12                                    12
#define DR13                                    13
#define DR14                                    14
#define DR15                                    15

#define MAJOR_VERSION3                          0

#define LAST_NIBBLE                             0x0F
#define FIRST_NIBBLE                            0xF0

// bit mask for MAC commands
// link ADR and RX2 setup
#define CHANNEL_MASK_ACK                        0x01
#define DATA_RATE_ACK                           0x02
#define RX1_DR_OFFSET_ACK                       0x04
#define POWER_ACK                               0x04

#define FPORT_MIN                               1
#define FPORT_MAX                               223

#define MAX_NB_CMD_TO_PROCESS                   16

//13 = sizeof(MIC) + MHDR + FHDR + sizeof (fPort);
#define HDRS_MIC_PORT_MIN_SIZE 13

#define ABP_TIMEOUT_MS                          20

#define JA_APP_NONCE_SIZE                       3
#define JA_JOIN_NONCE_SIZE                      3
#define JA_DEV_NONCE_SIZE                       2
#define JA_NET_ID_SIZE                          3

#define MAX_FOPTS_LEN                           0x0F

#define VIRTUAL_TIMER_TIMEOUT                   60000



typedef enum
{
    LORA_OK                                       = 0,
    NETWORK_NOT_JOINED                          ,
    MAC_STATE_NOT_READY_FOR_TRANSMISSION        ,
    INVALID_PARAMETER                           ,
    KEYS_NOT_INITIALIZED                        ,
    SILENT_IMMEDIATELY_ACTIVE                   ,
    FRAME_COUNTER_ERROR_REJOIN_NEEDED           ,
    INVALID_BUFFER_LENGTH                       ,
    MAC_PAUSED                                  ,
    NO_CHANNELS_FOUND                           ,
    INVALID_CLASS                               ,
    MCAST_PARAM_ERROR                           ,
    MCAST_MSG_ERROR                             ,
    DEVICE_DEVNONCE_ERROR                       ,
} LorawanError_t;

typedef enum
{
    MAC_NOT_OK = 0,     //LoRaWAN operation failed
    MAC_OK,             //LoRaWAN operation successful
    RADIO_NOT_OK,       //Radio operation failed
    RADIO_OK,           //Radio operation successful
    INVALID_BUFFER_LEN, //during retransmission, we have changed SF and the buffer is too large
    MCAST_RE_KEYING_NEEDED
} OpStatus_t;

typedef enum
{
    OTAA = 0,     //LoRaWAN Over The Air Activation - OTAA
    ABP           //LoRaWAN Activation By Personalization - ABP
} ActivationType_t;

typedef enum
{
    UNCNF = 0, //LoRaWAN Unconfirmed Transmission
    CNF        //LoRaWAN Confirmed Transmission
} TransmissionType_t;

typedef enum
{
    ISM_EU868,
    ISM_EU433,
    ISM_RU864
} IsmBand_t;

typedef enum
{
    CLASS_A = 0,
    CLASS_B,
    CLASS_C,
} LoRaClass_t;

typedef union
{
    uint32_t value;
    struct
    {
        unsigned macState :4;                        //determines the state of transmission (rx window open, between tx and rx, etc)
        unsigned networkJoined :1;                   //if set, the network is joined
        unsigned automaticReply :1;                  //if set, ACK and uplink packets sent due to  FPending will be sent immediately
        unsigned adr :1;                             //if set, adaptive data rate is requested by server or application
        unsigned silentImmediately :1;               //if set, the Mac command duty cycle request was received
        unsigned macPause :1;                        //if set, the mac Pause function was called. LoRa modulation is not possible
        unsigned rxDone :1;                          //if set, data is ready for reception
        unsigned linkCheck :1;                       //if set, linkCheck mechanism is enabled
        unsigned channelsModified :1;                //if set, new channels are added via CFList or NewChannelRequest command or enabled/disabled via Link Adr command
        unsigned txPowerModified :1;                 //if set, the txPower was modified via Link Adr command
        unsigned nbRepModified :1;                   //if set, the number of repetitions for unconfirmed frames has been modified
        unsigned prescalerModified :1;               //if set, the prescaler has changed via duty cycle request
        unsigned secondReceiveWindowModified :1;     //if set, the second receive window parameters have changed
        unsigned rxTimingSetup :1;                   //if set, the delay between the end of the TX uplink and the opening of the first reception slot has changed
        unsigned rejoinNeeded :1;                    //if set, the device must be rejoined as a frame counter issue happened
        unsigned mcastEnable :1;                     //if set, the device is in multicast mode and can receive multicast messages
    };
} LorawanStatus_t;

typedef union
{
    uint16_t value;
    struct
    {
        unsigned ackRequiredFromNextDownlinkMessage:1;  //if set, the next downlink message should have the ACK bit set because an ACK is needed for the end device
        unsigned ackRequiredFromNextUplinkMessage:1;    //if set, the next uplink message should have the ACK bit set because an ACK is needed for the server
        unsigned joining: 1;
        unsigned fPending:1;
        unsigned adrAckRequest:1;
        unsigned synchronization:1;                     //if set, there is no need to send immediately a packet because the application sent one from the callback
    };
} LorawanMacStatus_t;

typedef union
{
    uint32_t value;
    uint8_t buffer[4];
} DeviceAddress_t;

//activation parameters
typedef union
{
    uint32_t value;
    struct
    {
        uint16_t valueLow;
        uint16_t valueHigh;
    } members;
} FCnt_t;

typedef union
{
    uint8_t value;
    struct
    {
        uint8_t rx2DataRate     : 4;
        uint8_t rx1DROffset     : 3;
        uint8_t rfu             : 1;
    }bits;
} DlSettings_t;

typedef struct
{
    GenericEui_t Eui;
    uint16_t DevNonce;
} EEPROM_Data_t;

typedef union
{
    uint8_t value;
    struct
    {
        uint8_t joining : 1;
        uint8_t joined  : 1;
        uint8_t rfu     : 6;
    } states;
} DeviceStatus_t;

typedef union
{
    uint8_t value;
    struct
    {
       unsigned nbRep:4;
       unsigned chMaskCntl:3;
       unsigned rfu:1;
    };
} Redundancy_t;

typedef enum
{
    IDLE                      =0,
    TRANSMISSION_OCCURRING      ,
    BEFORE_RX1                  ,         //between TX and RX1, FSK can occur
    RX1_OPEN                    ,
    BETWEEN_RX1_RX2             ,         //FSK can occur
    RX2_OPEN                    ,
    RETRANSMISSION_DELAY        ,         //used for ADR_ACK delay, FSK can occur
    ABP_DELAY                   ,         //used for delaying in calling the join callback for ABP
    CLASS_C_RX2_1_OPEN          ,
    CLASS_C_RX2_2_OPEN          ,
    BEFORE_ACK                  ,
    BEFORE_TX1                  ,
    RXCONT                      ,
} LoRaMacState_t;

// types of frames
typedef enum
{
    FRAME_TYPE_JOIN_REQ         =0x00 ,
    FRAME_TYPE_JOIN_ACCEPT            ,
    FRAME_TYPE_DATA_UNCONFIRMED_UP    ,
    FRAME_TYPE_DATA_UNCONFIRMED_DOWN  ,
    FRAME_TYPE_DATA_CONFIRMED_UP      ,
    FRAME_TYPE_DATA_CONFIRMED_DOWN    ,
    FRAME_TYPE_RFU                    ,
    FRAME_TYPE_PROPRIETARY            ,
}LoRaMacFrameType_t;

// MAC commands CID
typedef enum
{
    LINK_CHECK_CID              = 0x02,
    LINK_ADR_CID                = 0x03,
    DUTY_CYCLE_CID              = 0x04,
    RX2_SETUP_CID               = 0x05,
    DEV_STATUS_CID              = 0x06,
    NEW_CHANNEL_CID             = 0x07,
    RX_TIMING_SETUP_CID         = 0x08,
}LoRaMacCid_t;

typedef struct
{
    ActivationType_t activationType;
    DeviceAddress_t deviceAddress;
    uint8_t networkSessionKey[16];
    uint8_t applicationSessionKey[16];
    uint8_t applicationKey[16];
    GenericEui_t applicationEui;
    GenericEui_t deviceEui;
    GenericEui_t joinEui;
    DeviceAddress_t mcastDeviceAddress;
    uint8_t mcastNetworkSessionKey[16];
    uint8_t mcastApplicationSessionKey[16];
} ActivationParameters_t;

typedef union
{
    uint8_t value;
    struct
    {
       unsigned fOptsLen:4;
       unsigned fPending:1;
       unsigned ack:1;
       unsigned adrAckReq:1;
       unsigned adr:1;
    };
} FCtrl_t;

// Mac header structure
typedef union
{
    uint8_t value;
    struct
    {
        uint8_t major           : 2;
        uint8_t rfu             : 3;
        uint8_t mType           : 3;
    }bits;
} Mhdr_t;

typedef union
{
    uint8_t fHdrCounter[23];
    struct
    {
        Mhdr_t mhdr             ;
        DeviceAddress_t devAddr ;
        FCtrl_t fCtrl           ;
        uint16_t fCnt           ;
        uint8_t MacCommands[15] ;
    }members;
} Hdr_t;

//Protocol parameters
typedef struct
{
    uint16_t receiveDelay1     ;
    uint16_t receiveDelay2     ;
    uint16_t joinAcceptDelay1  ;
    uint16_t joinAcceptDelay2  ;
    uint16_t maxFcntGap        ;
    uint16_t maxMultiFcntGap   ;
    uint16_t ackTimeout        ;
    uint8_t adrAckLimit        ;
    uint8_t adrAckDelay        ;
} ProtocolParams_t;

typedef struct
{
    uint8_t receivedCid;
    unsigned channelMaskAck :1;             // used for link adr answer
    unsigned dataRateAck :1;                // used for link adr answer
    unsigned powerAck :1;                   // used for link adr answer
    unsigned channelAck :1;                 // used for RX param setup request
    unsigned dataRateReceiveWindowAck :1;   // used for RX param setup request
    unsigned rx1DROffestAck :1;             // used for RX param setup request
    unsigned dataRateRangeAck :1;           // used for new channel answer
    unsigned channelFrequencyAck :1;        // used for new channel answer
} LorawanCommands_t;

typedef union
{
    uint16_t value;
    struct
    {
        unsigned deviceEui: 1;              //if set, device EUI was defined
        unsigned applicationEui:1;
        unsigned joinEui:1;
        unsigned deviceAddress: 1;
        unsigned applicationKey:1;
        unsigned networkSessionKey:1;
        unsigned applicationSessionKey:1;
        unsigned mcastApplicationSessionKey:1;
        unsigned mcastNetworkSessionKey:1;
        unsigned mcastDeviceAddress:1;
    };
} LorawanMacKeys_t;

typedef struct
{
    uint32_t frequency;
    uint8_t dataRate;
} ReceiveWindowParameters_t;

// minimum and maximum data rate
typedef union
{
    uint8_t value;
    struct
    {
       unsigned min:4;
       unsigned max:4;
    };
} DataRange_t;

typedef struct
{
    GenericEui_t Eui;
    uint16_t DevNonce;
    uint8_t js;
    uint8_t NwkSKey[16];
    uint8_t AppSKey[16];
    DeviceAddress_t DevAddr;
    DlSettings_t DlSettings;
    uint8_t rxDelay;
//    uint8_t cfList[16];
    FCnt_t fCntUp;
    FCnt_t fCntDown;
    DeviceStatus_t status;
    uint8_t sendJoinAccept1TimerId;
    uint8_t sendWindow1TimerId;
    LorawanMacStatus_t lorawanMacStatus;
    LorawanStatus_t macStatus;
    uint8_t macBuffer[32];
    uint8_t bufferIndex;
} Profile_t;

typedef union
{
    uint8_t joinAcceptCounter[29];
    struct
    {
        Mhdr_t mhdr;
        uint8_t joinNonce[3];
        uint8_t networkId[3];
        DeviceAddress_t deviceAddress;
        DlSettings_t DLSettings;
        uint8_t rxDelay;
        uint8_t cfList[16];
    } members;
} JoinAccept_t;

typedef union
{
    uint8_t joinRequestCounter[19];
    struct
    {
        Mhdr_t mhdr;
        GenericEui_t JoinEui;
        GenericEui_t DevEui;
        uint16_t devNonce;
    } members;
} JoinRequest_t;

//Channel parameters
typedef struct
{
        uint32_t frequency;
        bool status;
        DataRange_t dataRange;
        uint16_t dutyCycle;
        uint32_t channelTimer;
        bool joinRequestChannel;
        uint8_t parametersDefined;
} ChannelParams_t;

typedef struct
{
    LorawanMacStatus_t lorawanMacStatus;
    LorawanStatus_t macStatus;
    FCnt_t fCntUp;
    FCnt_t fCntDown;
    FCnt_t fMcastCntDown;
    LoRaClass_t deviceClass;
    ReceiveWindowParameters_t receiveWindow1Parameters;
    ReceiveWindowParameters_t receiveWindow2Parameters;
    ActivationParameters_t activationParameters;
    ChannelParams_t channelParameters;
    ProtocolParams_t protocolParameters;
    IsmBand_t ismBand;
    LorawanMacKeys_t macKeys;
    uint8_t crtMacCmdIndex;
    LorawanCommands_t macCommands[MAX_NB_CMD_TO_PROCESS];
    uint32_t lastTimerValue;
    uint32_t periodForLinkCheck;
    uint16_t adrAckCnt;
    uint16_t devNonce;
    uint16_t lastPacketLength;
    uint8_t maxRepetitionsUnconfirmedUplink;
    uint8_t maxRepetitionsConfirmedUplink;
    uint8_t counterRepetitionsUnconfirmedUplink;
    uint8_t counterRepetitionsConfirmedUplink;
    uint8_t lastUsedChannelIndex;
    uint16_t prescaler;
    uint8_t linkCheckMargin;
    uint8_t linkCheckGwCnt;
    uint8_t currentDataRate;
    uint8_t batteryLevel;
    uint8_t txPower;
    uint8_t joinAccept1TimerId;
    uint8_t joinAccept2TimerId;
    uint8_t receiveWindow1TimerId;
    uint8_t receiveWindow2TimerId;
    uint8_t automaticReplyTimerId;
    uint8_t linkCheckTimerId;
    uint8_t ackTimeoutTimerId;
    uint8_t dutyCycleTimerId;
    uint8_t unconfirmedRetransmisionTimerId;
    uint8_t minDataRate;
    uint8_t maxDataRate;
    uint8_t maxChannels;
    uint8_t counterAdrAckDelay;
    uint8_t offset;
    bool macInitialized;
    bool rx2DelayExpired;
    bool abpJoinStatus;
    uint8_t abpJoinTimerId;
    uint8_t syncWord;
    uint8_t sendDownAck1TimerId;
    uint8_t sendJoinAccept1TimerId;
    uint8_t virtualTimer;
} LoRa_t;


#endif /* COMPONENTS_LORAWAN_LORAWAN_TYPES_H_ */
