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
  uint8_t net_len[4];
} AmsTcpHdrType;

typedef struct AmsNetidAndPortType {
  uint8_t netID[6];
  uint8_t port_low;
  uint8_t port_high;
} AmsNetidAndPortType;



typedef struct {
  AmsTcpHdrType amsTcpHdr;
  AmsNetidAndPortType target;
  AmsNetidAndPortType source;
  uint8_t cmdID_low;
  uint8_t cmdID_high;
  uint8_t stateFlags_low;
  uint8_t stateFlags_high;
  uint8_t net_len[4];
  uint8_t net_errCode[4];
  uint8_t net_invokeID[4];
} AmsHdrType;

typedef struct {
  AmsHdrType amsHdr;
  uint8_t net_idxGrp[4];
  uint8_t net_idxOff[4];
  uint8_t net_len[4];
} AdsReadReqType;

typedef struct {
  AmsHdrType amsHdr;
  uint8_t net_idxGrp[4];
  uint8_t net_idxOff[4];
  uint8_t net_len[4];
} AdsWriteReqType;

typedef struct {
  AmsHdrType amsHdr;
  uint8_t net_idxGrp[4];
  uint8_t net_idxOff[4];
  uint8_t net_rd_len[4];
  uint8_t net_wr_len[4];
} AdsReadWriteReqType;

typedef struct {
  AmsHdrType amsHdr;
  struct {
    uint8_t net_res[4];
    uint8_t major;
    uint8_t minor;
    uint8_t versionBuild_low;
    uint8_t versionBuild_high;
    char    deviceName[16];
  } response;
} AdsReadDeviceInfoRepType;

typedef struct {
  AmsHdrType amsHdr;
  struct {
    uint8_t net_res[4];
    uint8_t net_len[4];
  } response;
} AdsReadRepType;

typedef struct {
  AmsHdrType amsHdr;
  struct {
    uint8_t net_res[4];
    uint8_t net_len[4];
  } response;
} AdsReadWriteRepType;


typedef struct {
  AmsHdrType amsHdr;
  struct {
    uint8_t net_res[4];
  } response;
} AdsWriteRepType;

typedef struct {
  uint8_t entryLen[4];
  uint8_t indexGroup[4];
  uint8_t indexOffset[4];
  uint8_t size[4];
  uint8_t dataType[4];
  uint8_t flags[4];
  uint8_t nameLength[2];
  uint8_t typeLength[2];
  uint8_t commentLength[2];
  char    buffer[768]; //256*3, 256 is string size in TwinCAT then 768 is max
} AdsSymbolInfoType;

typedef struct {
  AdsReadWriteRepType adsReadWriteRep;
  AdsSymbolInfoType   symbol_info;
} AdsGetSymbolInfoByNameRepType;
#endif
