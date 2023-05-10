#include <string.h>
#include <sys/socket.h>
#include "logerr_info.h"
#include "sock-util.h"
#include "indexer.h"
#include "cmd_EAT.h"
#include "ads.h"
#include "logerr_info.h"
#include "cmd_Sim_Ads.h"

#define ADSIGRP_SYM_INFOBYNAMEEX 0xF009

static simulatedNetworkProblemType
  simulatedNetworkProblemNew = simulatedNetworkProblemNone;


void handleADSread(int fd, ams_hdr_type *ams_hdr_p)
{
  ads_read_req_type *ads_read_req_p = (ads_read_req_type *)ams_hdr_p;
  size_t total_len_reply;
  size_t payload_len;
  ADS_Read_rep_type *ADS_Read_rep_p;
  ADS_Read_rep_p = (ADS_Read_rep_type *)ams_hdr_p;
  uint16_t adsport = ams_hdr_p->target.port_low + (ams_hdr_p->target.port_high << 8);

  uint32_t indexGroup = (uint32_t)ads_read_req_p->indexGroup_0 +
                        (ads_read_req_p->indexGroup_1 << 8) +
                        (ads_read_req_p->indexGroup_2 << 16) +
                        (ads_read_req_p->indexGroup_3 << 24);
  uint32_t indexOffset = (uint32_t)ads_read_req_p->indexOffset_0 +
                        (ads_read_req_p->indexOffset_1 << 8) +
                        (ads_read_req_p->indexOffset_2 << 16) +
                        (ads_read_req_p->indexOffset_3 << 24);
  uint32_t len_in_PLC = (uint32_t)ads_read_req_p->lenght_0 +
                        (ads_read_req_p->lenght_1 << 8) +
                        (ads_read_req_p->lenght_2 << 16) +
                        (ads_read_req_p->lenght_3 << 24);
  payload_len      = sizeof(ADS_Read_rep_p->response) + len_in_PLC;
  total_len_reply = sizeof(*ADS_Read_rep_p) -
                    sizeof(ADS_Read_rep_p->response) + payload_len;

  memset(&ADS_Read_rep_p->response, 0, sizeof(ADS_Read_rep_p->response));

  LOGINFO7("%s/%s:%d ADS_Readcmd indexGroup=0x%x indexOffset=%u len_in_PLC=%u payload_len=%u total_len_reply=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           indexGroup, indexOffset,len_in_PLC,
           (unsigned)payload_len, (unsigned)total_len_reply);
  ADS_Read_rep_p->response.lenght_0 = (uint8_t)(len_in_PLC);
  ADS_Read_rep_p->response.lenght_1 = (uint8_t)(len_in_PLC >> 8);
  ADS_Read_rep_p->response.lenght_2 = (uint8_t)(len_in_PLC >> 16);
  ADS_Read_rep_p->response.lenght_3 = (uint8_t)(len_in_PLC >> 24);
  indexerHandlePLCcycle();
  {
    uint8_t *data_ptr = (uint8_t *)ADS_Read_rep_p + sizeof(*ADS_Read_rep_p);
    (void)indexerHandleADS_ADR_getMemory(adsport, indexOffset,
                                         len_in_PLC, data_ptr);
    LOGINFO7("%s/%s:%d ADS_Readcmd ADS_Read_rep_p=%p data_ptr=%p data=0x%2x 0x%2x 0x%2x 0x%2x\n",
             __FILE__,__FUNCTION__, __LINE__,
             ADS_Read_rep_p, data_ptr,
             data_ptr[0],
             data_ptr[1],
             data_ptr[2],
             data_ptr[3]
             );
    if (len_in_PLC > 50) {
      /* Only for poll() readings - but not for "special device" */
      send_ams_reply_simulate_network_problem(fd,
                                              ams_hdr_p,
                                              total_len_reply,
                                              simulatedNetworkProblemNew);
      if (simulatedNetworkProblemNew != simulatedNetworkProblemNone) {
        simulatedNetworkProblemNew = simulatedNetworkProblemNone;
      }
    } else {
      send_ams_reply(fd, ams_hdr_p, total_len_reply);
    }
  }
}

static double netToDouble(void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float    fRes;
    } dst;
    dst.uRes = (uint32_t)src[0] + ((uint32_t)src[1] << 8) +
      ((uint32_t)src[2] << 16) + ((uint32_t)src[3] << 24);
    return (double)dst.fRes;
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double   fRes;
    } dst;
    dst.uRes = (uint64_t)src[0] + ((uint64_t)src[1] << 8) +
      ((uint64_t)src[2] << 16) + ((uint64_t)src[3] << 24) +
      ((uint64_t)src[4] << 32) + ((uint64_t)src[5] << 40) +
      ((uint64_t)src[6] << 48) + ((uint64_t)src[7] << 56);
    return dst.fRes;
  } else {
    return 0.0;
  }
}

void handleADSwrite(int fd, ams_hdr_type *ams_hdr_p)
{
  ADS_Write_req_type *ADS_Write_req_p = (ADS_Write_req_type *)ams_hdr_p;
  ADS_Write_rep_type *ADS_Write_rep_p = (ADS_Write_rep_type *)ams_hdr_p;
  uint16_t adsport = ams_hdr_p->target.port_low + (ams_hdr_p->target.port_high << 8);

  uint32_t indexGroup = (uint32_t)ADS_Write_req_p->indexGroup_0 +
                        (ADS_Write_req_p->indexGroup_1 << 8) +
                        (ADS_Write_req_p->indexGroup_2 << 16) +
                        (ADS_Write_req_p->indexGroup_3 << 24);
  uint32_t indexOffset = (uint32_t)ADS_Write_req_p->indexOffset_0 +
                        (ADS_Write_req_p->indexOffset_1 << 8) +
                        (ADS_Write_req_p->indexOffset_2 << 16) +
                        (ADS_Write_req_p->indexOffset_3 << 24);
  uint32_t len_in_PLC = (uint32_t)ADS_Write_req_p->lenght_0 +
                        (ADS_Write_req_p->lenght_1 << 8) +
                        (ADS_Write_req_p->lenght_2 << 16) +
                        (ADS_Write_req_p->lenght_3 << 24);

  memset(&ADS_Write_rep_p->response, 0, sizeof(ADS_Write_rep_p->response));

  LOGINFO7("%s/%s:%d ADS_Writecmd adsport=%u indexGroup=0x%x indexOffset=%u len_in_PLC=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           adsport, indexGroup, indexOffset,len_in_PLC);
  if (adsport == 852 && indexGroup == 0x4020) {
    if (len_in_PLC == 2) {
      uint8_t *data_ptr = (uint8_t *)ADS_Write_req_p + sizeof(*ADS_Write_req_p);
      unsigned value;
      value = data_ptr[0] + (data_ptr [1] << 8);

      LOGINFO7("%s/%s:%d ADS_Writecmd data=0x%x 0x%x value=0x%x\n",
               __FILE__,__FUNCTION__, __LINE__,
               data_ptr[0], data_ptr[1], value);
      indexerHandleADS_ADR_putUInt(adsport, indexOffset,
                                   len_in_PLC, value);
    } else {
      uint8_t *data_ptr = (uint8_t *)ADS_Write_req_p + sizeof(*ADS_Write_req_p);
      (void)indexerHandleADS_ADR_setMemory(adsport, indexOffset,
                                           len_in_PLC, data_ptr);
    }
    send_ams_reply(fd, ams_hdr_p, sizeof(ADS_Write_rep_type));
  } else if ((adsport == 501) && (len_in_PLC == 2)) {
    uint8_t *data_ptr = (uint8_t *)ADS_Write_req_p + sizeof(*ADS_Write_req_p);
    unsigned value = data_ptr[0] + (data_ptr [1] << 8);
    int ret = motorHandleADS_ADR_putInt(adsport, indexGroup, indexOffset,
                                        (int)value);
    LOGINFO3("%s/%s:%d ADS_Writecmd adsport=%u indexGroup=0x%x indexOffset=%u"
             " value=%u len_in_PLC=%u ret=%d\n",
             __FILE__,__FUNCTION__, __LINE__,
             adsport, indexGroup, indexOffset, value, len_in_PLC, ret);
    send_ams_reply(fd, ams_hdr_p, sizeof(ADS_Write_rep_type));
  } else if ((adsport == 501) && (len_in_PLC == 8)) {
    uint8_t *data_ptr = (uint8_t *)ADS_Write_req_p + sizeof(*ADS_Write_req_p);
    double fValue;
    fValue = netToDouble(data_ptr, len_in_PLC);
    int ret = motorHandleADS_ADR_putFloat(adsport, indexGroup, indexOffset,
                                          fValue);
    LOGINFO3("%s/%s:%d ADS_Writecmd adsport=%u indexGroup=0x%x indexOffset=%u"
             " value=%f len_in_PLC=%u ret=%d\n",
             __FILE__,__FUNCTION__, __LINE__,
             adsport, indexGroup, indexOffset, fValue, len_in_PLC, ret);
    send_ams_reply(fd, ams_hdr_p, sizeof(ADS_Write_rep_type));
  } else {
    LOGERR("%s/%s:%d Error: illegal combination adsport=%u indexGroup=0x%x indexOffset=%u len_in_PLC=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           adsport, indexGroup, indexOffset,len_in_PLC);
    if (DIE_ON_ERROR_BIT1())  exit(2);
  }
}


void handleADSreadwrite(int fd, ams_hdr_type *ams_hdr_p)
{
  ADS_ReadWrite_req_type *ADS_ReadWrite_req_p = (ADS_ReadWrite_req_type *)ams_hdr_p;
  ADS_ReadWrite_rep_type *ADS_ReadWrite_rep_p = (ADS_ReadWrite_rep_type *)ams_hdr_p;
  uint16_t adsport = ams_hdr_p->target.port_low + (ams_hdr_p->target.port_high << 8);

  uint32_t indexGroup = (uint32_t)ADS_ReadWrite_req_p->indexGroup_0 +
                        (ADS_ReadWrite_req_p->indexGroup_1 << 8) +
                        (ADS_ReadWrite_req_p->indexGroup_2 << 16) +
                        (ADS_ReadWrite_req_p->indexGroup_3 << 24);
  uint32_t indexOffset = (uint32_t)ADS_ReadWrite_req_p->indexOffset_0 +
                        (ADS_ReadWrite_req_p->indexOffset_1 << 8) +
                        (ADS_ReadWrite_req_p->indexOffset_2 << 16) +
                        (ADS_ReadWrite_req_p->indexOffset_3 << 24);
  uint32_t rd_len_in_PLC = (uint32_t)ADS_ReadWrite_req_p->rd_len_0 +
                           (ADS_ReadWrite_req_p->rd_len_1 << 8) +
                           (ADS_ReadWrite_req_p->rd_len_2 << 16) +
                           (ADS_ReadWrite_req_p->rd_len_3 << 24);

  uint32_t wr_len_in_PLC = (uint32_t)ADS_ReadWrite_req_p->wr_len_0 +
                           (ADS_ReadWrite_req_p->wr_len_1 << 8) +
                           (ADS_ReadWrite_req_p->wr_len_2 << 16) +
                           (ADS_ReadWrite_req_p->wr_len_3 << 24);

  memset(ADS_ReadWrite_rep_p, 0, sizeof(*ADS_ReadWrite_rep_p));

  LOGINFO7("%s/%s:%d ADS_ReadWritecmd indexGroup=0x%x indexOffset=%u rd_len_in_PLC=%u wr_len_in_PLC=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           indexGroup, indexOffset, rd_len_in_PLC, wr_len_in_PLC);
  if (indexGroup == ADSIGRP_SYM_INFOBYNAMEEX) {
    LOGINFO7("%s/%s:%d ADS_ReadWritecmd ADSIGRP_SYM_INFOBYNAMEEX (%s)\n",
           __FILE__,__FUNCTION__, __LINE__,
             ADS_ReadWrite_req_p->data
           );
  }


  if (wr_len_in_PLC == 2) {
    unsigned value;
    value = ADS_ReadWrite_req_p->data[0] +
      (ADS_ReadWrite_req_p->data[1] << 8);

    LOGINFO7("%s/%s:%d ADS_ReadWritecmd data=0x%x 0x%x value=0x%x\n",
             __FILE__,__FUNCTION__, __LINE__,
             ADS_ReadWrite_req_p->data[0],
             ADS_ReadWrite_req_p->data[1],value);
    indexerHandleADS_ADR_putUInt(adsport,
                                 indexOffset,
                                 wr_len_in_PLC,
                                 value);
  } else {
    (void)indexerHandleADS_ADR_setMemory(adsport,
                                         indexOffset,
                                         wr_len_in_PLC,
                                         &ADS_ReadWrite_req_p->data);
  }
}

void send_ams_reply(int fd, ams_hdr_type *ams_hdr_p, uint32_t total_len_reply)
{
  send_ams_reply_simulate_network_problem(fd, ams_hdr_p, total_len_reply,
                                          simulatedNetworkProblemNone);
}

void
send_ams_reply_simulate_network_problem(int fd, ams_hdr_type *ams_hdr_p,
                                        uint32_t total_len_reply,
                                        simulatedNetworkProblemType simulatedNetworkProblem)
{
  uint32_t len_to_socket = total_len_reply;
  uint32_t ams_payload_len = total_len_reply - sizeof(*ams_hdr_p);
  uint32_t ams_tcp_header_len = total_len_reply - sizeof(ams_tcp_header_type);
  ams_netid_port_type ams_netid_port_tmp;

  LOGINFO7("%s/%s:%d total_len_reply=%u ams_tcp_header_len=%u ams_payload_len=%u id=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           total_len_reply, ams_tcp_header_len, ams_payload_len,
           ams_hdr_p->invokeID_0 +
           (ams_hdr_p->invokeID_1 << 8) +
           (ams_hdr_p->invokeID_2 << 16) +
           (ams_hdr_p->invokeID_3 << 24)
           );
  /* Swap source and target */
  memcpy(&ams_netid_port_tmp, &ams_hdr_p->target,  sizeof(ams_netid_port_type));
  memcpy(&ams_hdr_p->target,  &ams_hdr_p->source,  sizeof(ams_netid_port_type));
  memcpy(&ams_hdr_p->source,  &ams_netid_port_tmp, sizeof(ams_netid_port_type));

  if (simulatedNetworkProblemNew != simulatedNetworkProblemNone) {
    switch (simulatedNetworkProblem) {
    case simulatedNetworkProblemAmsTcpHdrOnly:
    case simulatedNetworkProblemAmsTcpHdrOnlyEOF:
      len_to_socket = sizeof(ams_tcp_header_type);
      break;
    case simulatedNetworkProblemAmsTcpHdrShortOnly:
    case simulatedNetworkProblemAmsTcpHdrShortOnlyEOF:
      len_to_socket = sizeof(ams_tcp_header_type) - 1;
      break;
    case simulatedNetworkProblemPacketTooShort:
    case simulatedNetworkProblemPacketTooShortEOF:
      len_to_socket = total_len_reply - 1;
      break;
    case simulatedNetworkProblemAmsHeaderLengthTooShort:
      ams_tcp_header_len -= 1;
      break;
    case simulatedNetworkProblemAmsHeaderLengthTooLong:
      ams_tcp_header_len += 1;
      break;
    case simulatedNetworkProblemInvokeID_0:
      ams_hdr_p->invokeID_0++;
      break;
    case simulatedNetworkProblemInvokeID_1:
      ams_hdr_p->invokeID_1++;
      break;
    case simulatedNetworkProblemInvokeID_2:
      ams_hdr_p->invokeID_2++;
      break;
    case simulatedNetworkProblemInvokeID_3:
      ams_hdr_p->invokeID_3++;
      break;
    default:
      ;
    }
    LOGERR("%s/%s:%d ADS_Readcmd ADS_new_simulatedNetworkProblemNew=%u len_to_socket=%u (0x%02x) total_len_reply=%u (0x%02x)\n",
           __FILE__,__FUNCTION__, __LINE__,
           (int)simulatedNetworkProblemNew,
           (unsigned)len_to_socket, (unsigned)len_to_socket,
           (unsigned)total_len_reply, (unsigned)total_len_reply);
  }

  ams_hdr_p->ams_tcp_header.lenght_0 = (uint8_t)ams_tcp_header_len;
  ams_hdr_p->ams_tcp_header.lenght_1 = (uint8_t)(ams_tcp_header_len >> 8);
  ams_hdr_p->ams_tcp_header.lenght_2 = (uint8_t)(ams_tcp_header_len >> 16);
  ams_hdr_p->ams_tcp_header.lenght_3 = (uint8_t)(ams_tcp_header_len >> 24);
  ams_hdr_p->stateFlags_low = 5;
  ams_hdr_p->stateFlags_high = 0;
  ams_hdr_p->lenght_0 = (uint8_t)ams_payload_len;
  ams_hdr_p->lenght_1 = (uint8_t)(ams_payload_len >> 8);
  ams_hdr_p->lenght_2 = (uint8_t)(ams_payload_len >> 16);
  ams_hdr_p->lenght_3 = (uint8_t)(ams_payload_len >> 24);

  LOGINFO7("%s/%s:%d ADS_Readcmd total_len_reply=%u ams_tcp_header_len=%u tcplen_0=%u tcplen_1=%u tcplen_2=%u tcplen_3=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
          (unsigned)total_len_reply,
          (unsigned)ams_tcp_header_len,
          ams_hdr_p->ams_tcp_header.lenght_0,
          ams_hdr_p->ams_tcp_header.lenght_1,
          ams_hdr_p->ams_tcp_header.lenght_2,
          ams_hdr_p->ams_tcp_header.lenght_3);

  LOGINFO7("%s/%s:%d ADS_Readcmd total_len_reply=%u ams_payload_len=%u lenght_0=%u lenght_1=%u lenght_2=%u lenght_3=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
          (unsigned)total_len_reply,
          (unsigned)ams_payload_len,
          ams_hdr_p->lenght_0,
          ams_hdr_p->lenght_1,
          ams_hdr_p->lenght_2,
          ams_hdr_p->lenght_3);

  send_to_socket(fd, ams_hdr_p, len_to_socket);
  switch (simulatedNetworkProblem) {
    case simulatedNetworkProblemShutdownRead:
      {
        int ret = shutdown(fd, SHUT_RD);
        LOGERR("%s/%s:%d shutdown(%d, SHUT_RD) ret=%d\n",
               __FILE__,__FUNCTION__, __LINE__,
               fd, ret);
      }
      break;
    case simulatedNetworkProblemAmsTcpHdrOnlyEOF:
    case simulatedNetworkProblemAmsTcpHdrShortOnlyEOF:
    case simulatedNetworkProblemPacketTooShortEOF:
      {
        int ret = shutdown(fd, SHUT_WR);
        LOGERR("%s/%s:%d shutdown(%d, SHUT_WR) ret=%d\n",
               __FILE__,__FUNCTION__, __LINE__,
               fd, ret);
      }
      break;
    default:
      ;
  }
}

static int adsHandleOneArg(const char *myarg_1)
{
  static const char * const Sim_this_ads_dot_str = "Sim.this.ads.";
  const char *myarg = myarg_1;
  unsigned simulated_network_problem = 0;
  int nvals;

  /* Sim.this.ads. */
  if (!strncmp(myarg_1, Sim_this_ads_dot_str, strlen(Sim_this_ads_dot_str))) {
    myarg_1 += strlen(Sim_this_ads_dot_str);
  }
  /* From here on, only  Sim.this.ads. commands */
  nvals = sscanf(myarg_1, "simulatedNetworkProblem=%u", &simulated_network_problem);
  if (nvals != 1) {
    LOGERR("%s/%s:%d line=%s nvals=%d myarg_1=\"%s\"",
           __FILE__, __FUNCTION__, __LINE__,
           myarg, nvals, myarg_1);
    exit(2);
  }
  if (simulated_network_problem < simulatedNetworkProblemLast) {
    simulatedNetworkProblemNew = (simulatedNetworkProblemType)simulated_network_problem;
    return 0;
  } else {
    LOGERR("%s/%s:%d line=%s nvals=%d simulated_network_problem=%u out of range. Allowed: 0..%u myarg_1=\"%s\"",
           __FILE__, __FUNCTION__, __LINE__,
           myarg, nvals,
           simulated_network_problem,
           (unsigned)simulatedNetworkProblemLast - 1,
           myarg_1);
    return 1;
  }

  /* if we come here, we do not understand the command */
  LOGERR("%s/%s:%d illegal line=%s myarg_1=%s",
         __FILE__, __FUNCTION__, __LINE__,
         myarg, myarg_1);
  return 1;
}


int cmd_Sim_Ads(int argc, const char *argv[])
{
  const char *myargline = (argc > 0) ? argv[0] : "";
  int retval = 0;
  if (PRINT_STDOUT_BIT6())
  {
    const char *myarg[5];
    myarg[0] = myargline;
    myarg[1] = (argc > 1) ? argv[1] : "";
    myarg[2] = (argc > 2) ? argv[2] : "";
    myarg[3] = (argc > 3) ? argv[3] : "";
    myarg[4] = (argc > 4) ? argv[4] : "";
    LOGINFO6("%s/%s:%d argc=%d "
             "myargline=\"%s\" myarg[1]=\"%s\" myarg[2]=\"%s\" myarg[3]=\"%s\" myarg[4]=\"%s\"\n",
             __FILE__, __FUNCTION__, __LINE__,
             argc,  myargline,
             myarg[1], myarg[2],
             myarg[3], myarg[4]);
  }

  while (argc > 1) {
    retval |= adsHandleOneArg(argv[1]);
    argc--;
    argv++;
  } /* while argc > 0 */
  return retval;
}
