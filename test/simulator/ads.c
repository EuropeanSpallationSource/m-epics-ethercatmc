#include <string.h>
#include "logerr_info.h"
#include "sock-util.h"
#include "indexer.h"
#include "cmd_EAT.h"
#include "ads.h"

#define ADSIGRP_SYM_INFOBYNAMEEX 0xF009


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
  ADS_Read_rep_p->response.lenght_1 = (uint8_t)(len_in_PLC << 8);
  ADS_Read_rep_p->response.lenght_2 = (uint8_t)(len_in_PLC << 16);
  ADS_Read_rep_p->response.lenght_3 = (uint8_t)(len_in_PLC << 24);
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
  }
  send_ams_reply(fd, ams_hdr_p, total_len_reply);
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
  if (adsport == 851 && indexGroup == 0x4020) {
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
  } else {
    LOGERR("%s/%s:%d ADS_Writecmd adsport=%u indexGroup=0x%x indexOffset=%u len_in_PLC=%u\n",
           __FILE__,__FUNCTION__, __LINE__,
           adsport, indexGroup, indexOffset,len_in_PLC);
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

  ams_hdr_p->ams_tcp_header.lenght_0 = (uint8_t)ams_tcp_header_len;
  ams_hdr_p->ams_tcp_header.lenght_1 = (uint8_t)(ams_tcp_header_len >> 8);
  ams_hdr_p->ams_tcp_header.lenght_2 = (uint8_t)(ams_tcp_header_len >> 16);
  ams_hdr_p->ams_tcp_header.lenght_3 = (uint8_t)(ams_tcp_header_len >> 24);
  ams_hdr_p->stateFlags_low = 5;
  ams_hdr_p->stateFlags_high = 0;
  ams_hdr_p->lenght_0 = (uint8_t)ams_payload_len;
  ams_hdr_p->lenght_1 = (uint8_t)(ams_payload_len << 8);
  ams_hdr_p->lenght_2 = (uint8_t)(ams_payload_len << 16);
  ams_hdr_p->lenght_3 = (uint8_t)(ams_payload_len << 24);
  send_to_socket(fd, ams_hdr_p, total_len_reply);
}
