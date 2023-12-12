#include "sock-ads.h"

#include <inttypes.h>
#include <string.h>

#include "ads.h"
#include "indexer.h"
#include "logerr_info.h"
#include "sock-util.h"

size_t handle_ams_request(int fd, char *buf, size_t len, size_t buff_len_max) {
  ams_hdr_type *ams_hdr_p = (ams_hdr_type *)buf;

  uint16_t cmdId = ams_hdr_p->cmdID_low + (ams_hdr_p->cmdID_high << 8);
  uint32_t ams_lenght = ams_hdr_p->ams_tcp_header.lenght_0 +
                        (ams_hdr_p->ams_tcp_header.lenght_1 << 8) +
                        (ams_hdr_p->ams_tcp_header.lenght_2 << 16) +
                        (ams_hdr_p->ams_tcp_header.lenght_3 << 24);

  LOGINFO7("%s/%s:%d len=%lu AMS tcp_header=%x %x ams_lenght=%u\n", __FILE__,
           __FUNCTION__, __LINE__, (unsigned long)len,
           ams_hdr_p->ams_tcp_header.res0, ams_hdr_p->ams_tcp_header.res1,
           ams_lenght);

  LOGINFO7(
      "%s/%s:%d ams_header target=%d.%d.%d.%d.%d.%d:%d "
      "source=%d.%d.%d.%d.%d.%d:%d\n",
      __FILE__, __FUNCTION__, __LINE__, ams_hdr_p->target.netID[0],
      ams_hdr_p->target.netID[1], ams_hdr_p->target.netID[2],
      ams_hdr_p->target.netID[3], ams_hdr_p->target.netID[4],
      ams_hdr_p->target.netID[5],
      ams_hdr_p->target.port_low + ams_hdr_p->target.port_high * 256,
      ams_hdr_p->source.netID[0], ams_hdr_p->source.netID[1],
      ams_hdr_p->source.netID[2], ams_hdr_p->source.netID[3],
      ams_hdr_p->source.netID[4], ams_hdr_p->source.netID[5],
      ams_hdr_p->source.port_low + ams_hdr_p->source.port_high * 256);
  LOGINFO7("%s/%s:%d ams_header cmd=%u flags=%u len=%u err=%u id=%u\n",
           __FILE__, __FUNCTION__, __LINE__, cmdId,
           ams_hdr_p->stateFlags_low + (ams_hdr_p->stateFlags_high << 8),
           ams_hdr_p->lenght_0 + (ams_hdr_p->lenght_1 << 8) +
               (ams_hdr_p->lenght_2 << 16) + (ams_hdr_p->lenght_3 << 24),
           ams_hdr_p->errorCode_0 + (ams_hdr_p->errorCode_1 << 8) +
               (ams_hdr_p->errorCode_2 << 16) + (ams_hdr_p->errorCode_3 << 24),
           ams_hdr_p->invokeID_0 + (ams_hdr_p->invokeID_1 << 8) +
               (ams_hdr_p->invokeID_2 << 16) + (ams_hdr_p->invokeID_3 << 24));

  if (cmdId == ADS_READ_DEVICE_INFO) {
    const static char *const deviceName = "Simulator";
    uint32_t strlen_deviceName = strlen(deviceName);
    ADS_Read_Device_Info_rep_type *ADS_Read_Device_Info_rep_p;
    uint32_t total_len_reply;

#if 0
    total_len_reply =
      sizeof(*ADS_Read_Device_Info_rep_p) -
      sizeof(*ADS_Read_Device_Info_rep_p->deviceName) +
      strlen_deviceName;
#else
    total_len_reply = sizeof(*ADS_Read_Device_Info_rep_p);
#endif

    ADS_Read_Device_Info_rep_p = (ADS_Read_Device_Info_rep_type *)ams_hdr_p;
    memset(&ADS_Read_Device_Info_rep_p->response, 0,
           sizeof(ADS_Read_Device_Info_rep_p->response));
    memset(ADS_Read_Device_Info_rep_p->response.deviceName, ' ',
           sizeof(ADS_Read_Device_Info_rep_p->response.deviceName));

    ADS_Read_Device_Info_rep_p->response.major = 3;
    ADS_Read_Device_Info_rep_p->response.minor = 1;
    ADS_Read_Device_Info_rep_p->response.versionBuild_low = 10;
    ADS_Read_Device_Info_rep_p->response.versionBuild_high = 11;
    if (strlen_deviceName <
        sizeof(*ADS_Read_Device_Info_rep_p->response.deviceName)) {
      memcpy(ADS_Read_Device_Info_rep_p->response.deviceName, deviceName,
             strlen(deviceName));
    }
    send_ams_reply(fd, ams_hdr_p, total_len_reply);
    return len;
  } else if (cmdId == ADS_READ) {
    handleADSread(fd, ams_hdr_p);
    return len;
  } else if (cmdId == ADS_WRITE) {
    handleADSwrite(fd, ams_hdr_p);
    indexerHandlePLCcycle();
    return len;
  } else if (cmdId == ADS_READ_WRITE) {
    handleADSreadwrite(fd, ams_hdr_p);
    RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d command not implemented =%u",
                        __FILE__, __FUNCTION__, __LINE__, cmdId);
  } else {
    RETURN_ERROR_OR_DIE(__LINE__, "%s/%s:%d command not implemented =%u",
                        __FILE__, __FUNCTION__, __LINE__, cmdId);
  }

  return 0;  // len;
}
