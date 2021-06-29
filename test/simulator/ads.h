#ifndef AMS_H
#define AMS_H

#include <inttypes.h>
#include <stddef.h>

#define ADS_READ_DEVICE_INFO  1
#define ADS_READ              2
#define ADS_WRITE             3
#define ADS_READ_WRITE        9


/* AMS/TCP Header */
typedef struct {
  uint8_t res0;
  uint8_t res1;
  uint8_t lenght_0;
  uint8_t lenght_1;
  uint8_t lenght_2;
  uint8_t lenght_3;
} ams_tcp_header_type;

typedef struct ams_netid_port_type {
  uint8_t netID[6];
  uint8_t port_low;
  uint8_t port_high;
} ams_netid_port_type;



typedef struct {
  ams_tcp_header_type ams_tcp_header;
  ams_netid_port_type target;
  ams_netid_port_type source;
  uint8_t cmdID_low;
  uint8_t cmdID_high;
  uint8_t stateFlags_low;
  uint8_t stateFlags_high;
  uint8_t lenght_0;
  uint8_t lenght_1;
  uint8_t lenght_2;
  uint8_t lenght_3;
  uint8_t errorCode_0;
  uint8_t errorCode_1;
  uint8_t errorCode_2;
  uint8_t errorCode_3;
  uint8_t invokeID_0;
  uint8_t invokeID_1;
  uint8_t invokeID_2;
  uint8_t invokeID_3;
} ams_hdr_type;

typedef struct {
  ams_hdr_type ams_hdr;
  uint8_t indexGroup_0;
  uint8_t indexGroup_1;
  uint8_t indexGroup_2;
  uint8_t indexGroup_3;
  uint8_t indexOffset_0;
  uint8_t indexOffset_1;
  uint8_t indexOffset_2;
  uint8_t indexOffset_3;
  uint8_t lenght_0;
  uint8_t lenght_1;
  uint8_t lenght_2;
  uint8_t lenght_3;
} ads_read_req_type;

typedef struct {
  ams_hdr_type ams_hdr;
  uint8_t indexGroup_0;
  uint8_t indexGroup_1;
  uint8_t indexGroup_2;
  uint8_t indexGroup_3;
  uint8_t indexOffset_0;
  uint8_t indexOffset_1;
  uint8_t indexOffset_2;
  uint8_t indexOffset_3;
  uint8_t lenght_0;
  uint8_t lenght_1;
  uint8_t lenght_2;
  uint8_t lenght_3;
} ADS_Write_req_type;

typedef struct {
  ams_hdr_type ams_hdr;
  uint8_t indexGroup_0;
  uint8_t indexGroup_1;
  uint8_t indexGroup_2;
  uint8_t indexGroup_3;
  uint8_t indexOffset_0;
  uint8_t indexOffset_1;
  uint8_t indexOffset_2;
  uint8_t indexOffset_3;
  uint8_t rd_len_0;
  uint8_t rd_len_1;
  uint8_t rd_len_2;
  uint8_t rd_len_3;
  uint8_t wr_len_0;
  uint8_t wr_len_1;
  uint8_t wr_len_2;
  uint8_t wr_len_3;
  uint8_t data[256];
} ADS_ReadWrite_req_type;

typedef struct {
  ams_hdr_type ams_hdr;
  struct {
    uint8_t result_0;
    uint8_t result_1;
    uint8_t result_2;
    uint8_t result_3;
    uint8_t major;
    uint8_t minor;
    uint8_t versionBuild_low;
    uint8_t versionBuild_high;
    char    deviceName[16];
  } response;
} ADS_Read_Device_Info_rep_type;

typedef struct {
  ams_hdr_type ams_hdr;
  struct {
    uint8_t result_0;
    uint8_t result_1;
    uint8_t result_2;
    uint8_t result_3;
    uint8_t lenght_0;
    uint8_t lenght_1;
    uint8_t lenght_2;
    uint8_t lenght_3;
  } response;
} ADS_Read_rep_type;

typedef struct {
  ams_hdr_type ams_hdr;
  uint8_t result_0;
  uint8_t result_1;
  uint8_t result_2;
  uint8_t result_3;
  uint8_t lenght_0;
  uint8_t lenght_1;
  uint8_t lenght_2;
  uint8_t lenght_3;
} ADS_ReadWrite_rep_type;


typedef struct {
  ams_hdr_type ams_hdr;
  struct {
    uint8_t result_0;
    uint8_t result_1;
    uint8_t result_2;
    uint8_t result_3;
  } response;
} ADS_Write_rep_type;

typedef enum {
    simulatedNetworkProblemNone = 0,
    simulatedNetworkProblemAmsTcpHeaderOnly,
    simulatedNetworkProblemAmsTcpHdrShortOnly,
    simulatedNetworkProblemPacketTooShort,
    simulatedNetworkProblemAmsHeaderLengthTooShort,
    simulatedNetworkProblemAmsHeaderLengthTooLong,
    simulatedNetworkProblemInvokeID_0,
    simulatedNetworkProblemInvokeID_1,
    simulatedNetworkProblemInvokeID_2,
    simulatedNetworkProblemInvokeID_3,

    simulatedNetworkProblemLast
} simulatedNetworkProblemType;


void send_ams_reply(int fd, ams_hdr_type *ams_hdr_p, uint32_t total_len_reply);
void
send_ams_reply_simulate_network_problem(int fd, ams_hdr_type *ams_hdr_p,
                                        uint32_t total_len_reply,
                                        simulatedNetworkProblemType simulatedNetworkProblem);
void handleADSread(int fd, ams_hdr_type *ams_hdr_p);
void handleADSwrite(int fd, ams_hdr_type *ams_hdr_p);
void handleADSreadwrite(int fd, ams_hdr_type *ams_hdr_p);


#endif
