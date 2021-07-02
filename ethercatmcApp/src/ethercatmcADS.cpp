#include <stdlib.h>
#include <string.h>
#include "ethercatmcController.h"
#include "ethercatmcADSdefs.h"
#include <asynOctetSyncIO.h>

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#ifndef ASYN_TRACE_DEBUG
#define ASYN_TRACE_DEBUG     0x0080
#endif

//#define DEFAULT_CONTROLLER_TIMEOUT 2.0
#define MAXCNTADSSTATUS 10

static uint32_t invokeID;
static int deftracelevel = ASYN_TRACE_DEBUG;

#define ethercatmchexdump(pasynUser, tracelevel, help_txt, bufptr, buflen)\
{\
  const void* buf = (const void*)bufptr;\
  int len = (int)buflen;\
  uint8_t *data = (uint8_t *)buf;\
  int count;\
  unsigned pos = 0;\
  while (len > 0) {\
    struct {\
      char asc_txt[8];\
      char space[2];\
      char hex_txt[8][3];\
      char nul;\
    } print_buf;\
    memset(&print_buf, ' ', sizeof(print_buf));\
    print_buf.nul = '\0';\
    for (count = 0; count < 8; count++) {\
      if (count < len) {\
        unsigned char c = (unsigned char)data[count];\
        if (c >= 0x20 && c < 0x7F)\
          print_buf.asc_txt[count] = c;\
        else\
          print_buf.asc_txt[count] = '.';\
        snprintf((char*)&print_buf.hex_txt[count],\
                 sizeof(print_buf.hex_txt[count]),\
                 "%02x", c);\
        /* Replace NUL with ' ' after snprintf */\
        print_buf.hex_txt[count][2] = ' ';\
      }\
    }\
    asynPrint(pasynUser, tracelevel,\
              "%s%s [%02x]%s\n",\
              modNamEMC, help_txt, pos, (char*)&print_buf);\
    len -= 8;\
    data += 8;\
    pos += 8;\
  }\
}\


#define ethercatmcamsdump(pasynUser, tracelevel, help_txt, ams_headdr_p)\
{\
  const AmsHdrType *amsHdr_p = (const AmsHdrType *)(ams_headdr_p);\
  unsigned amsTcpHdr_len = amsHdr_p->amsTcpHdr.net_len[0] +\
    (amsHdr_p->amsTcpHdr.net_len[1] << 8) +\
    (amsHdr_p->amsTcpHdr.net_len[2] << 16) +\
    (amsHdr_p->amsTcpHdr.net_len[3] <<24);\
    unsigned ams_lenght = amsHdr_p->net_len[0] +\
      (amsHdr_p->net_len[1] << 8) +\
      (amsHdr_p->net_len[2] << 16) +\
      (amsHdr_p->net_len[3] << 24);\
    unsigned ams_errorCode = amsHdr_p->net_errCode[0] +\
      (amsHdr_p->net_errCode[1] << 8) +\
      (amsHdr_p->net_errCode[2] << 16) +\
      (amsHdr_p->net_errCode[3] << 24);\
    unsigned ams_invokeID = amsHdr_p->net_invokeID[0] +\
      (amsHdr_p->net_invokeID[1] << 8) +\
      (amsHdr_p->net_invokeID[2] << 16) +\
      (amsHdr_p->net_invokeID[3] << 24);\
    unsigned cmd = amsHdr_p->cmdID_low + (amsHdr_p->cmdID_high <<8); \
  asynPrint(pasynUser, tracelevel,\
            "%samsTcpHdr_len=%u ams target=%d.%d.%d.%d.%d.%d:%d "  \
            "source=%d.%d.%d.%d.%d.%d:%d\n",                       \
            help_txt, (unsigned)amsTcpHdr_len,                               \
            amsHdr_p->target.netID[0], amsHdr_p->target.netID[1],\
            amsHdr_p->target.netID[2], amsHdr_p->target.netID[3],\
            amsHdr_p->target.netID[4], amsHdr_p->target.netID[5],\
            amsHdr_p->target.port_low + (amsHdr_p->target.port_high << 8),\
            amsHdr_p->source.netID[0],  amsHdr_p->source.netID[1],\
            amsHdr_p->source.netID[2],  amsHdr_p->source.netID[3],\
            amsHdr_p->source.netID[4],  amsHdr_p->source.netID[5],\
            amsHdr_p->source.port_low + (amsHdr_p->source.port_high << 8)\
            );\
  asynPrint(pasynUser, tracelevel,\
            "%samsHdr cmd=%u flags=%u ams_len=%u ams_err=%u id=%u\n",\
            help_txt,                                        \
            cmd,\
            amsHdr_p->stateFlags_low + (amsHdr_p->stateFlags_high << 8),\
            ams_lenght, ams_errorCode, ams_invokeID);\
  if (cmd == ADS_READ || cmd == ADS_WRITE || cmd == ADS_READ_WRITE) { \
    AdsReadReqType *ads_read_req_p = (AdsReadReqType*)ams_headdr_p;\
    const char *cmd_str = "";\
    switch (cmd) { case ADS_READ: cmd_str =  "RD"; break;\
                   case ADS_WRITE: cmd_str = "WR"; break;\
                   case ADS_READ_WRITE: cmd_str = "WRRD"; break;\
                 }\
    unsigned idxGrp = ads_read_req_p->net_idxGrp[0] +\
      (ads_read_req_p->net_idxGrp[1] << 8) +\
      (ads_read_req_p->net_idxGrp[2] << 16) +\
      (ads_read_req_p->net_idxGrp[3] << 24);\
    unsigned idxOff = ads_read_req_p->net_idxOff[0] +\
      (ads_read_req_p->net_idxOff[1] << 8) + \
      (ads_read_req_p->net_idxOff[2] << 16) +\
      (ads_read_req_p->net_idxOff[3] << 24);\
    unsigned len = ads_read_req_p->net_len[0] +\
      (ads_read_req_p->net_len[1] << 8) + \
      (ads_read_req_p->net_len[2] << 16) +\
      (ads_read_req_p->net_len[3] << 24);\
  asynPrint(pasynUser, tracelevel,\
            "%s %4s idxGrp=0X%04X idxOff=0x%04X len=%u\n",\
            help_txt, cmd_str,                           \
            idxGrp, idxOff, len);                        \
  } \
}\

/****************************************************************************/
extern "C" unsigned netToUint(const void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  unsigned uRes;
  if (lenInPlc == 2) {
    uRes = (unsigned)src[0] + ((unsigned)src[1] << 8);
    return uRes;
  } else if ((lenInPlc == 4) || (lenInPlc == 8)) {
    /* We don't use the full range of 64 bit integers,
       only values up to 2^31 */
    uRes = (unsigned)src[0] + ((unsigned)src[1] << 8) +
           ((unsigned)src[2] << 16) + ((unsigned)src[3] << 24);
    return uRes;
  }
  return 0;
}

extern "C" int netToSint(const void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  if (lenInPlc == 2) {
    int16_t uRes16;
    uRes16 = (unsigned)src[0] + ((unsigned)src[1] << 8);
    return (int)(int16_t)uRes16; /* sign extend */
  } else if ((lenInPlc == 4) || (lenInPlc == 8)) {
    unsigned uRes32;
    /* We don't use the full range of 64 bit integers,
       only values up to 2^31 */
    uRes32 = (unsigned)src[0] + ((unsigned)src[1] << 8) +
           ((unsigned)src[2] << 16) + ((unsigned)src[3] << 24);
    return (int)uRes32;
  }
  return 0;
}

extern "C" uint64_t netToUint64(const void *data, size_t lenInPlc)
{
  const uint8_t *src = (const uint8_t*)data;
  uint64_t uRes;
  if (lenInPlc == 8) {
    uRes = (uint64_t)src[0] +
      ((uint64_t)src[1] << 8) +
      ((uint64_t)src[2] << 16) +
      ((uint64_t)src[3] << 24) +
      ((uint64_t)src[4] << 32) +
      ((uint64_t)src[5] << 40) +
      ((uint64_t)src[6] << 48) +
      ((uint64_t)src[7] << 56);
    return uRes;
  }
  return 0;
}

extern "C" int64_t netToSint64(const void *data, size_t lenInPlc)
{
  return (int64_t)netToUint64(data, lenInPlc);
}
extern "C" double netToDouble(const void *data, size_t lenInPlc)
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

extern "C" void doubleToNet(const double value, void *data, size_t lenInPlc)
{
  uint8_t *dst = (uint8_t*)data;
  if (lenInPlc == 4) {
    union {
      volatile uint32_t uRes;
      volatile float    fRes;
    } src;
    src.fRes = (float)value;
    dst[0] = (uint8_t)src.uRes;
    dst[1] = (uint8_t)(src.uRes >> 8);
    dst[2] = (uint8_t)(src.uRes >> 16);
    dst[3] = (uint8_t)(src.uRes >> 24);
  } else if (lenInPlc == 8) {
    union {
      volatile uint64_t uRes;
      volatile double   fRes;
    } src;
    src.fRes = value;
    dst[0] = (uint8_t)src.uRes;
    dst[1] = (uint8_t)(src.uRes >> 8);
    dst[2] = (uint8_t)(src.uRes >> 16);
    dst[3] = (uint8_t)(src.uRes >> 24);
    dst[4] = (uint8_t)(src.uRes >> 32);
    dst[5] = (uint8_t)(src.uRes >> 40);
    dst[6] = (uint8_t)(src.uRes >> 48);
    dst[7] = (uint8_t)(src.uRes >> 56);
  } else {
    memset(data, 0, lenInPlc);
  }
}

extern "C" void uintToNet(const unsigned value, void *data, size_t lenInPlc)
{
  uint8_t *dst = (uint8_t*)data;
  memset(data, 0, lenInPlc);
  if (lenInPlc == 2) {
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
  } else if ((lenInPlc == 4) || (lenInPlc == 8)) {
    /* We don't use the full range of 64 bit integers,
       only values up to 2^31 */
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
  }
}



asynStatus
ethercatmcController::writeReadControllerADS(asynUser *pasynUser,
                                             const char *outdata,
                                             size_t outlen,
                                             char *indata,
                                             size_t inlen,
                                             size_t *pnread,
                                             const char *fileName,
                                             int lineNo)
{
  int tracelevel = deftracelevel;
  int errorProblem = 0;
  int eomReason;
  size_t nwrite = 0;
  size_t nread1 = 0;
  size_t nread2 = 0;
  uint32_t part_1_len = sizeof(AmsTcpHdrType);
  asynStatus status;
  *pnread = 0;
  EMC_ENTER_ADS_CHECK_LOCK(lineNo);
  status = pasynOctetSyncIO->write(pasynUser, outdata, outlen,
                                   DEFAULT_CONTROLLER_TIMEOUT,
                                   &nwrite);
  if (nwrite != outlen) {
    if (ctrlLocal.cntADSstatus < MAXCNTADSSTATUS) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s:%d outlen=%lu nwrite=%lu timeout=%f err=%s status=%s (%d)\n",
                fileName, lineNo,
                (unsigned long)outlen,
                (unsigned long)nwrite,
                DEFAULT_CONTROLLER_TIMEOUT,
                pasynUser->errorMessage,
                ethercatmcstrStatus(status), status);
      ctrlLocal.cntADSstatus++;
    }
    EMC_LEAVE_ADS_CHECK_LOCK(__LINE__);
    return asynError;
  }
  ethercatmchexdump(pasynUser, tracelevel, "OUT",
                    outdata, outlen);
  /* Read the AMS/TCP Header */
  status = pasynOctetSyncIO->read(pasynUser,
                                  indata, part_1_len,
                                  DEFAULT_CONTROLLER_TIMEOUT,
                                  &nread1, &eomReason);
  if (status) errorProblem |= 1;
  if (!nread1) errorProblem |= 2;
  if (eomReason & ASYN_EOM_END) errorProblem |= 4;
  if (errorProblem) tracelevel |= ASYN_TRACE_ERROR;

  asynPrint(pasynUser, tracelevel,
            "%s:%d IN1 toread=0x%x %u nread1=%lu eomReason=%x (%s%s%s) err=%s errorProblem=%d status=%s (%d)\n",
            fileName, lineNo,
            (unsigned)part_1_len, (unsigned)part_1_len, (unsigned long)nread1,
            eomReason,
            eomReason & ASYN_EOM_CNT ? "CNT" : "",
            eomReason & ASYN_EOM_EOS ? "EOS" : "",
            eomReason & ASYN_EOM_END ? "END" : "",
            pasynUser->errorMessage,
            errorProblem,
            ethercatmcstrStatus(status), status);

  /* asyn returns nread == 0 and asynSuccess on EOF */
  if (eomReason & ASYN_EOM_END) status = asynError;
  if (!errorProblem && !status) {
    /* The length to read is inside the AMS/TCP header */
    const AmsHdrType *amsHdr_p = (const AmsHdrType *)indata;
    uint32_t toread = NETTOUINT(amsHdr_p->amsTcpHdr.net_len);

    /* Read the rest into indata */
    status = pasynOctetSyncIO->read(pasynUser,
                                    indata + part_1_len,
                                    toread,
                                    DEFAULT_CONTROLLER_TIMEOUT,
                                    &nread2, &eomReason);
    if (status) errorProblem |= 1;
    if (!nread2) errorProblem |= 2;
    if (eomReason & ASYN_EOM_END) errorProblem |= 4;
    if (errorProblem) tracelevel |= ASYN_TRACE_ERROR;
    asynPrint(pasynUser, tracelevel,
              "%s:%d IN2 toread=0x%x %u nread2=%lu eomReason=%x (%s%s%s) status=%s (%d)\n",
              fileName, lineNo,
              (unsigned)toread, (unsigned)toread,
              (unsigned long)nread2,
              eomReason,
              eomReason & ASYN_EOM_CNT ? "CNT" : "",
              eomReason & ASYN_EOM_EOS ? "EOS" : "",
              eomReason & ASYN_EOM_END ? "END" : "",
              ethercatmcstrStatus(status), status);
    if (eomReason & ASYN_EOM_END) status = asynError;
    *pnread = nread1 + nread2;
  }
  EMC_LEAVE_ADS_CHECK_LOCK(__LINE__);
  ethercatmcamsdump(pasynUser, tracelevel, "OUT", outdata);
  ethercatmchexdump(pasynUser, tracelevel, "OUT", outdata, outlen);
  if (nread2) {
    ethercatmcamsdump(pasynUser, tracelevel, "IN1 ", indata);
    ethercatmchexdump(pasynUser, tracelevel, "IN1 ", indata, nread1);
    ethercatmchexdump(pasynUser, tracelevel, "IN2 ", indata, nread1 + nread2);
  }

  if (status == asynTimeout) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s status == asynTimeout: calling disconnect_C\n", modNamEMC);
    disconnect_C(pasynUser);
  }
  return status ? asynError : asynSuccess;
}

asynStatus ethercatmcController::writeReadAds(asynUser *pasynUser,
                                              AmsHdrType *ams_req_hdr_p,
                                              size_t outlen,
                                              uint16_t targetAdsport,
                                              uint32_t invokeID,
                                              uint32_t ads_cmdID,
                                              void *indata, size_t inlen,
                                              size_t *pnread,
                                              const char *fileName,
                                              int lineNo)
{
  asynStatus status;
  uint32_t ams_payload_len = outlen - sizeof(ams_req_hdr_p->amsTcpHdr);
  uint32_t ads_len = outlen - sizeof(*ams_req_hdr_p);
  *pnread = 0;

  UINTTONET(ams_payload_len, ams_req_hdr_p->amsTcpHdr.net_len);
  memcpy(&ams_req_hdr_p->target.netID,
         &ctrlLocal.remote.netID,  sizeof(ams_req_hdr_p->target.netID));
  ams_req_hdr_p->target.port_low  = (uint8_t)targetAdsport;
  ams_req_hdr_p->target.port_high = (uint8_t)(targetAdsport >> 8);
  memcpy(&ams_req_hdr_p->source,
         &ctrlLocal.local, sizeof(ams_req_hdr_p->source));
  ams_req_hdr_p->cmdID_low  = (uint8_t)ads_cmdID;
  ams_req_hdr_p->cmdID_high = (uint8_t)(ads_cmdID >> 8);
  ams_req_hdr_p->stateFlags_low = 0x4; /* Command */
  UINTTONET(ads_len, ams_req_hdr_p->net_len);
  UINTTONET(invokeID, ams_req_hdr_p->net_invokeID);
  status = writeReadControllerADS(pasynUser,
                                  (const char *)ams_req_hdr_p,
                                  outlen,
                                  (char *)indata, inlen,
                                  pnread,
                                  fileName, lineNo);
  if (!status) {
    size_t nread = *pnread;
    AmsHdrType *ams_rep_hdr_p = (AmsHdrType*)indata;
    uint32_t amsTcpHdr_len_exp = nread - sizeof(ams_rep_hdr_p->amsTcpHdr);
    uint32_t amsTcpHdr_len_act = NETTOUINT(ams_rep_hdr_p->amsTcpHdr.net_len);

    if ((!status) && (amsTcpHdr_len_act != amsTcpHdr_len_exp)) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s:%d nread=%u calling disconnect_C: amsTcpHdr_len_exp=%u 0x%08X amsTcpHdr_len_act=%u 0x%08X\n",
                fileName, lineNo, (unsigned)nread,
                (unsigned)amsTcpHdr_len_exp, (unsigned)amsTcpHdr_len_exp,
                (unsigned)amsTcpHdr_len_act, (unsigned)amsTcpHdr_len_act);
      disconnect_C(pasynUser);
      status = asynError;
    }
    if (!status) {
      uint32_t ads_rep_len_exp = nread - sizeof(AmsHdrType);
      uint32_t ads_rep_len_act = NETTOUINT(ams_rep_hdr_p->net_len);
      if (ads_rep_len_act != ads_rep_len_exp) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s:%d nread=%u calling disconnect_C: ads_rep_len_exp=%u  0x%08X ads_rep_len_act=%u 0x%08X \n",
                  fileName, lineNo, (unsigned)nread,
                  (unsigned)ads_rep_len_exp, (unsigned)ads_rep_len_exp,
                  (unsigned)ads_rep_len_act, (unsigned)ads_rep_len_act);
        disconnect_C(pasynUser);
        status = asynError;
      }
    }
    if (!status) {
      uint32_t ams_errorCode = NETTOUINT(ams_rep_hdr_p->net_errCode);

      if (ams_errorCode) {
        int tracelevel = ASYN_TRACEIO_DRIVER;
        if (ctrlLocal.cntADSstatus < MAXCNTADSSTATUS) {
          tracelevel |= ASYN_TRACE_ERROR;
          ctrlLocal.cntADSstatus++;
        }

        const char *outdata = (const char *)ams_req_hdr_p;
        #define ERR_TARGETPORTNOTFOUND 6
        #define ERR_TARGETMACHINENOTFOUND 7
        switch (ams_errorCode) {
        case ERR_TARGETPORTNOTFOUND:
        case ERR_TARGETMACHINENOTFOUND:
          asynPrint(pasynUser, tracelevel,
                    "%s:%d Error: MACHINE/PORT not found ams_errorCode=0x%x\n",
                    fileName, lineNo,
                    (unsigned)ams_errorCode);
          return asynDisabled;
        default:
          ;
        }
        ethercatmchexdump(pasynUser, tracelevel, "OUT", outdata, outlen);
        ethercatmcamsdump(pasynUser, tracelevel, "OUT", ams_req_hdr_p);
        ethercatmchexdump(pasynUser, tracelevel, "IN ", indata, nread);
        ethercatmcamsdump(pasynUser, tracelevel, "IN ", indata);

        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s nread=%u ams_errorCode=0x%x\n", modNamEMC,
                  (unsigned)nread, (unsigned)ams_errorCode);
        status = asynError;
      }
    }
    if (!status) {
      uint32_t rep_invokeID = NETTOUINT(ams_rep_hdr_p->net_invokeID);
      if (invokeID != rep_invokeID) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s:%d calling disconnect_C: invokeID=%u 0x%08X rep_invokeID=%u 0x%08X\n",
                  fileName, lineNo,
                  (unsigned)invokeID, (unsigned)invokeID,
                  (unsigned)rep_invokeID, (unsigned)rep_invokeID);
        disconnect_C(pasynUser);
        status = asynError;
      }
    }
  }
  return status;
}

asynStatus ethercatmcController::getPlcMemoryViaADSFL(unsigned indexOffset,
                                                    void *data,
                                                    size_t lenInPlc,
                                                    const char *fileName,
                                                    int lineNo)
{
  static unsigned indexGroup = 0x4020;
  int tracelevel = deftracelevel;
  asynUser *pasynUser = pasynUserController_;
  AdsReadReqType ads_read_req;

  size_t read_buf_len = sizeof(AdsReadRepType) + lenInPlc;
  void *p_read_buf = malloc(read_buf_len);

  asynStatus status;
  size_t nread = 0;

  memset(&ads_read_req, 0, sizeof(ads_read_req));
  memset(p_read_buf, 0, read_buf_len);
  invokeID++;

  UINTTONET(indexGroup, ads_read_req.net_idxGrp);
  UINTTONET(indexOffset, ads_read_req.net_idxOff);
  UINTTONET(lenInPlc, ads_read_req.net_len);

  status = writeReadAds(pasynUser,
                        (AmsHdrType *)&ads_read_req,
                        sizeof(ads_read_req),
                        ctrlLocal.adsport,
                        invokeID, ADS_READ,
                        (char*)p_read_buf, read_buf_len,
                        &nread, fileName, lineNo);
  asynPrint(pasynUser, tracelevel,
            "%s:%d RDMEM indexOffset=%u lenInPlc=%u status=%s (%d)\n",
            fileName, lineNo, indexOffset, (unsigned)lenInPlc,
            ethercatmcstrStatus(status), (int)status);
  if (!status)
  {
    AdsReadRepType *ADS_Read_rep_p = (AdsReadRepType*) p_read_buf;
    uint32_t ads_result = netToUint(&ADS_Read_rep_p->response,
                                    sizeof(ADS_Read_rep_p->response.net_res));
    uint32_t ads_length = netToUint(&ADS_Read_rep_p->response.net_len,
                                    sizeof(ADS_Read_rep_p->response.net_len));
    if (ads_result) {
      int tracelevel = ASYN_TRACEIO_DRIVER;
      if (ctrlLocal.cntADSstatus < MAXCNTADSSTATUS) {
        tracelevel |= ASYN_TRACE_ERROR;
        ctrlLocal.cntADSstatus++;
      }
      asynPrint(pasynUser, tracelevel,
                "%sads_result=0x%x\n", modNamEMC, (unsigned)ads_result);
      status = asynError;
    } else if (ads_length != lenInPlc) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s:%d lenInPlc=%lu ads_length=%u\n",
                  fileName, lineNo,
                  (unsigned long)lenInPlc,(unsigned)ads_length);
        status = asynError;
    }
    if (!status) {
      uint8_t *src_ptr = (uint8_t*) p_read_buf;
      src_ptr += sizeof(AdsReadRepType);
      memcpy(data, src_ptr, ads_length);
      ethercatmchexdump(pasynUser, tracelevel, "RDMEM", src_ptr, ads_length);
    }
  }
  free(p_read_buf);
  return status;
}

asynStatus ethercatmcController::setPlcMemoryViaADSFL(unsigned indexOffset,
                                                      const void *data,
                                                      size_t lenInPlc,
                                                      const char *fileName,
                                                      int lineNo)
{
  const static unsigned indexGroup = 0x4020;
  return setMemIdxGrpIdxOffFL(indexGroup, indexOffset,
                              ctrlLocal.adsport,
                              data, lenInPlc,
                              fileName, lineNo);
}

asynStatus ethercatmcController::setMemIdxGrpIdxOffFL(unsigned indexGroup,
                                                      unsigned indexOffset,
                                                      unsigned targetAdsport,
                                                      const void *data,
                                                      size_t lenInPlc,
                                                      const char *fileName,
                                                      int lineNo)
{
  int tracelevel = deftracelevel;
  asynUser *pasynUser = pasynUserController_;
  AdsWriteRepType ADS_Write_rep;
  AdsWriteReqType *ads_write_req_p;

  size_t write_buf_len = sizeof(AdsWriteReqType) + lenInPlc;
  void *p_write_buf = malloc(write_buf_len);

  asynStatus status;
  size_t nread = 0;
  ads_write_req_p = (AdsWriteReqType *)p_write_buf;

  memset(p_write_buf, 0, write_buf_len);
  memset(&ADS_Write_rep, 0, sizeof(ADS_Write_rep));
  invokeID++;

  UINTTONET(indexGroup,  ads_write_req_p->net_idxGrp);
  UINTTONET(indexOffset, ads_write_req_p->net_idxOff);
  UINTTONET(lenInPlc,    ads_write_req_p->net_len);

  asynPrint(pasynUser, tracelevel,
            "%s:%d WR indexOffset=%u lenInPlc=%u\n",
            fileName, lineNo, indexOffset, (unsigned)lenInPlc
            );
  /* copy the payload */
  {
    uint8_t *dst_ptr = (uint8_t*)p_write_buf;
    dst_ptr += sizeof(AdsWriteReqType);
    memcpy(dst_ptr, data, lenInPlc);
    ethercatmchexdump(pasynUser, tracelevel, "WRMEM", data, lenInPlc);
  }
  status = writeReadAds(pasynUser,
                        (AmsHdrType *)p_write_buf, write_buf_len,
                        targetAdsport,
                        invokeID, ADS_WRITE,
                        &ADS_Write_rep, sizeof(ADS_Write_rep),
                        &nread, fileName,  lineNo);

  if (!status) {
    uint32_t ads_result = NETTOUINT(ADS_Write_rep.response.net_res);
    if (ads_result) {
      tracelevel |= ASYN_TRACE_INFO;
      ethercatmcamsdump(pasynUser, tracelevel, "OUT ", p_write_buf);
      ethercatmchexdump(pasynUser, tracelevel, "WRMEM", data, lenInPlc);

      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s:%d ads_result=0x%x\n",
                fileName, lineNo, (unsigned)ads_result);
      status = asynError;
    }
  }
  return status;
}


asynStatus ethercatmcController::getSymbolInfoViaADS(const char *symbolName,
                                                     void *data,
                                                     size_t lenInPlc)
{
  int tracelevel = deftracelevel;
  asynUser *pasynUser = pasynUserController_;
  unsigned indexGroup = 0xF009;
  unsigned indexOffset = 0;
  size_t   symbolNameLen = strlen(symbolName);

  size_t write_buf_len = sizeof(AdsReadWriteReqType) + symbolNameLen;
  size_t read_buf_len  = sizeof(AdsReadWriteRepType) + lenInPlc;
  void *p_read_buf = malloc(read_buf_len);
  AdsReadWriteReqType *ads_read_write_req_p =
    (AdsReadWriteReqType*)malloc(write_buf_len);

  asynStatus status;
  size_t nread = 0;

  memset(ads_read_write_req_p, 0, write_buf_len);
  memset(p_read_buf, 0, read_buf_len);
  memset(data, 0, lenInPlc);
  invokeID++;

  UINTTONET(indexGroup,    ads_read_write_req_p->net_idxGrp);
  UINTTONET(indexOffset,   ads_read_write_req_p->net_idxOff);
  UINTTONET(lenInPlc,      ads_read_write_req_p->net_rd_len);
  UINTTONET(symbolNameLen, ads_read_write_req_p->net_wr_len);

  /* copy the symbol name */
  {
    uint8_t *dst_ptr = (uint8_t*)ads_read_write_req_p;
    dst_ptr += sizeof(AdsReadWriteReqType);
    memcpy(dst_ptr, symbolName, symbolNameLen);
    ethercatmchexdump(pasynUser, tracelevel, "LOOKUP",
                      symbolName, symbolNameLen);
  }
  status = writeReadAds(pasynUser,
                        (AmsHdrType *)ads_read_write_req_p,
                        write_buf_len,
                        ctrlLocal.adsport,
                        invokeID, ADS_READ_WRITE,
                        (char*)p_read_buf, read_buf_len,
                        &nread,
                        __FILE__, __LINE__);
  if (!status)
  {
    AdsReadWriteRepType *adsReadWriteRep_p = (AdsReadWriteRepType*) p_read_buf;
    uint32_t ads_result = NETTOUINT(adsReadWriteRep_p->response.net_res);
    uint32_t ads_length = NETTOUINT(adsReadWriteRep_p->response.net_len);
    if (ads_result) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sERROR ads_result=0x%x\n", modNamEMC, (unsigned)ads_result);
      status = asynError;
    }
    if (!status) {
      AdsGetSymbolInfoByNameRepType *adsGgetSymbolInfoByName_rep_p;
      adsGgetSymbolInfoByName_rep_p = (AdsGetSymbolInfoByNameRepType *)p_read_buf;
      (void)adsGgetSymbolInfoByName_rep_p;
    }
    if (!status) {
      uint8_t *src_ptr = (uint8_t*) p_read_buf;
      src_ptr += sizeof(AdsReadWriteRepType);
      asynPrint(pasynUser, tracelevel,
                "%s ads_result=0x%x ads_length=0x%x\n",
                modNamEMC, (unsigned)ads_result, (unsigned)ads_length);
      ethercatmchexdump(pasynUser, tracelevel, "IN ADS", src_ptr, ads_length);
      memcpy(data, src_ptr, ads_length);
    }
  }
  free(ads_read_write_req_p);
  free(p_read_buf);
  return status;
}

asynStatus
ethercatmcController::getSymbolHandleByNameViaADS(const char *symbolName,
                                                  uint32_t *handle)
{
  int tracelevel = deftracelevel;
  asynUser *pasynUser = pasynUserController_;
  unsigned indexGroup = 0xF003;
  unsigned indexOffset = 0;
  size_t lenInPlc = sizeof(uint32_t);
  size_t symbolNameLen = strlen(symbolName);

  size_t write_buf_len = sizeof(AdsReadWriteReqType) + symbolNameLen;
  size_t read_buf_len  = sizeof(AdsReadWriteRepType) + lenInPlc;
  void *p_read_buf = malloc(read_buf_len);
  AdsReadWriteReqType *ads_read_write_req_p =
    (AdsReadWriteReqType*)malloc(write_buf_len);

  asynStatus status;
  size_t nread = 0;

  memset(ads_read_write_req_p, 0, write_buf_len);
  memset(p_read_buf, 0, read_buf_len);
  invokeID++;

  UINTTONET(indexGroup,    ads_read_write_req_p->net_idxGrp);
  UINTTONET(indexOffset,   ads_read_write_req_p->net_idxOff);
  UINTTONET(lenInPlc,      ads_read_write_req_p->net_rd_len);
  UINTTONET(symbolNameLen, ads_read_write_req_p->net_wr_len);

  /* copy the symbol name */
  {
    uint8_t *dst_ptr = (uint8_t*)ads_read_write_req_p;
    dst_ptr += sizeof(AdsReadWriteReqType);
    memcpy(dst_ptr, symbolName, symbolNameLen);
    ethercatmchexdump(pasynUser, tracelevel, "LOOKUP",
                      symbolName, symbolNameLen);
  }
  status = writeReadAds(pasynUser,
                        (AmsHdrType *)ads_read_write_req_p,
                        write_buf_len,
                        ctrlLocal.adsport,
                        invokeID, ADS_READ_WRITE,
                        (char*)p_read_buf, read_buf_len,
                        &nread,
                        __FILE__, __LINE__);
  if (!status)
  {
    AdsReadWriteRepType *adsReadWriteRep_p = (AdsReadWriteRepType*) p_read_buf;
    uint32_t ads_result = NETTOUINT(adsReadWriteRep_p->response.net_res);
    uint32_t ads_length = NETTOUINT(adsReadWriteRep_p->response.net_len);
    if (ads_result) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sERROR ads_result=0x%x\n", modNamEMC, (unsigned)ads_result);
      status = asynError;
    }
    if (!status) {
      AdsGetSymbolInfoByNameRepType *adsGgetSymbolInfoByName_rep_p;
      adsGgetSymbolInfoByName_rep_p = (AdsGetSymbolInfoByNameRepType *)p_read_buf;
      (void)adsGgetSymbolInfoByName_rep_p;
    }
    if (!status) {
      uint8_t *src_ptr = (uint8_t*) p_read_buf;
      src_ptr += sizeof(AdsReadWriteRepType);
      asynPrint(pasynUser, tracelevel,
                "%s ads_result=0x%x ads_length=0x%x\n",
                modNamEMC, (unsigned)ads_result, (unsigned)ads_length);
      ethercatmchexdump(pasynUser, tracelevel, "IN ADS", src_ptr, ads_length);
      if (ads_length == lenInPlc) {
        *handle = netToUint(src_ptr, lenInPlc);
      } else {
        status = asynError;
      }
    }
  }
  free(ads_read_write_req_p);
  free(p_read_buf);
  return status;
}
