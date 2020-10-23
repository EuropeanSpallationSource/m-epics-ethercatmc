/*
FILENAME...   ethercatmcController.h
*/

#ifndef ETHERCATMCCONTROLLER_H
#define ETHERCATMCCONTROLLER_H

#include "asynDriver.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "ethercatmcAxis.h"
#include "ethercatmcADSdefs.h"

#ifndef VERSION_INT
#  define VERSION_INT(V,R,M,P) ( ((V)<<24) | ((R)<<16) | ((M)<<8) | (P))
#endif

#define VERSION_INT_4_38            VERSION_INT(4,38,0,0)
#define ETHERCATMC_ASYN_VERSION_INT VERSION_INT(ASYN_VERSION,ASYN_REVISION,ASYN_MODIFICATION,0)
#if ETHERCATMC_ASYN_VERSION_INT >= VERSION_INT_4_38
#define ETHERCATMC_ASYN_ASYNPARAMINT64
#endif

#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif

#define ethercatmcErrString                  "MCUErr"
#define ethercatmcErrIdString                "ErrorID"
#define ethercatmcStatusCodeString           "StatusCode"
#define ethercatmcStatusBitsString           "StatusBits"
#define ethercatmcNamAux0_String             "NamAuxBit0"
#define ethercatmcNamAux1_String             "NamAuxBit1"
#define ethercatmcNamAux2_String             "NamAuxBit2"
#define ethercatmcNamAux3_String             "NamAuxBit3"
#define ethercatmcNamAux4_String             "NamAuxBit4"
#define ethercatmcNamAux5_String             "NamAuxBit5"
#define ethercatmcNamAux6_String             "NamAuxBit6"
#define ethercatmcNamAux7_String             "NamAuxBit7"
#define ethercatmcNamBit24_String            "NamBit24"
#define ethercatmcNamBit25_String            "NamBit25"
#define ethercatmcFoffVisString              "FoffVis"
#define ethercatmcHomProc_RBString           "HomProc-RB"
#define ethercatmcHomPos_RBString            "HomPos-RB"
#define ethercatmcHomProcString              "HomProc"
#define ethercatmcHomPosString               "HomPos"
#define ethercatmcVelToHomString             "VelToHom"
#define ethercatmcVelFrmHomString            "VelFrmHom"
#define ethercatmcAccHomString               "AccHom"
#define ethercatmcEnc_ActString              "EncAct"
#define ethercatmcErrRstString               "ErrRst"
#define ethercatmcVelActString               "VelAct"
#define ethercatmcVel_RBString               "Vel-RB"
#define ethercatmcAcc_RBString               "Acc-RB"
#define ethercatmcCfgVELO_String             "CfgVELO"
#define ethercatmcCfgVMAX_String             "CfgVMAX"
#define ethercatmcCfgJVEL_String             "CfgJVEL"
#define ethercatmcCfgACCS_String             "CfgACCS"
#define ethercatmcCfgDHLMRBString            "CfgDHLM-RB"
#define ethercatmcCfgDLLMRBString            "CfgDLLM-RB"
#define ethercatmcCfgDHLM_EnRBString         "CfgDHLM-En-RB"
#define ethercatmcCfgDLLM_EnRBString         "CfgDLLM-En-RB"
#define ethercatmcCfgDHLMString              "CfgDHLM"
#define ethercatmcCfgDLLMString              "CfgDLLM"
#define ethercatmcCfgDHLM_EnString           "CfgDHLM-En"
#define ethercatmcCfgDLLM_EnString           "CfgDLLM-En"
#define ethercatmcCfgSREV_RBString           "CfgSREV-RB"
#define ethercatmcCfgUREV_RBString           "CfgUREV-RB"
#define ethercatmcCfgPMIN_RBString           "CfgPMIN-RB"
#define ethercatmcCfgPMAX_RBString           "CfgPMAX-RB"
#define ethercatmcCfgSPDB_RBString           "CfgSPDB-RB"
#define ethercatmcCfgRDBD_RBString           "CfgRDBD-RB"
#define ethercatmcCfgRDBD_Tim_RBString       "CfgRDBD-Tim-RB"
#define ethercatmcCfgRDBD_En_RBString        "CfgRDBD-En-RB"
#define ethercatmcCfgPOSLAG_RBString         "CfgPOSLAG-RB"
#define ethercatmcCfgPOSLAG_Tim_RBString     "CfgPOSLAG-Tim-RB"
#define ethercatmcCfgPOSLAG_En_RBString      "CfgPOSLAG-En-RB"
#define ethercatmcCfgDESC_RBString           "CfgDESC-RB"
#define ethercatmcCfgEGU_RBString            "CfgEGU-RB"


#define ethercatmcMCUErrMsgString            "MCUErrMsg"
#define ethercatmcDbgStrToMcuString          "StrToMCU"
#define ethercatmcDbgStrToLogString          "StrToLOG"

#define HOMPROC_MANUAL_SETPOS    15

extern const char *modNamEMC;

extern "C" {
  /* Struct to handle additional (PILS) devices.
     Create a conversion table, to map the PILS devices
     into the asynParameter library and vice-versa */
  typedef struct {
    char           *paramName;
    int            axisNo;      /* 0 is the controller */
    unsigned       inputOffset; /* offset inside the "plc memory bytes" */
    unsigned       outputOffset; /* offset inside the "plc memory bytes" */
    unsigned       lenInPLC;    /* len  inside the "plc memory bytes" */
    int            function;    /* asyn: "function" */
    asynParamType  myEPICSParamType; /* asynParamType.h */
    asynParamType  myMCUParamType; /* asynParamType.h */
    int            isSystemDClock:1;
  } pilsAsynDevInfo_type;
}
extern "C" {
  unsigned   netToUint(const void *data, size_t lenInPlc);
  double     netToDouble(const void *data, size_t lenInPlc);
  uint64_t   netToUint64(const void *data, size_t lenInPlc);
  void       doubleToNet(const double value, void *data, size_t lenInPlc);
  void       uintToNet(const unsigned value, void *data, size_t lenInPlc);
  int ethercatmcCreateAxis(const char *ethercatmcName, int axisNo,
                           int axisFlags, const char *axisOptionsStr);

  asynStatus ethercatmcADSgetPlcMemoryUint(asynUser *pasynUser,
                                           unsigned indexOffset,
                                           unsigned *value,
                                           size_t lenInPlc);
  asynStatus disconnect_C(asynUser *pasynUser);
  asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                          const char *outdata, size_t outlen,
                                          char *indata, size_t inlen);
  asynStatus checkACK(const char *outdata, size_t outlen, const char *indata);
  const char *plcUnitTxtFromUnitCode(unsigned unitCode);
  const char *plcParamIndexTxtFromParamIndex(unsigned paramIndex);
  const char *ethercatmcstrStatus(asynStatus status);
  const char *errStringFromErrId(int nErrorId);
}
#define NETTOUINT(n)       netToUint((const void*)&n, sizeof(n))
#define NETTODOUBLE(n)     netToDouble((const void*)&n, sizeof(n))
#define UINTTONET(val,n)   uintToNet((val), (&n), sizeof(n))
#define DOUBLETONET(val,n) doubleToNet((val), (&n), sizeof(n))

class ethercatmcIndexerAxis;

class epicsShareClass ethercatmcController : public asynMotorController {
public:
#define PARAM_IDX_OPMODE_AUTO_UINT            1
#define PARAM_IDX_MICROSTEPS_UINT             2
#define PARAM_IDX_ABS_MIN_FLOAT              30
#define PARAM_IDX_ABS_MAX_FLOAT              31
#define PARAM_IDX_USR_MIN_FLOAT              32
#define PARAM_IDX_USR_MAX_FLOAT              33
#define PARAM_IDX_WRN_MIN_FLOAT              34
#define PARAM_IDX_WRN_MAX_FLOAT              35
#define PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT    55
#define PARAM_IDX_HYTERESIS_FLOAT            56
#define PARAM_IDX_REFSPEED_FLOAT             58
#define PARAM_IDX_VBAS_FLOAT                 59
#define PARAM_IDX_SPEED_FLOAT                60
#define PARAM_IDX_ACCEL_FLOAT                61
#define PARAM_IDX_IDLE_CURRENT_FLOAT         62
#define PARAM_IDX_MOVE_CURRENT_FLOAT         64
#define PARAM_IDX_MICROSTEPS_FLOAT           67
#define PARAM_IDX_STEPS_PER_UNIT_FLOAT       68
#define PARAM_IDX_HOME_POSITION_FLOAT        69
#define PARAM_IDX_FUN_REFERENCE             133
#define PARAM_IDX_FUN_SET_POSITION          137
#define PARAM_IDX_FUN_MOVE_VELOCITY         142


#define FEATURE_BITS_V1               (1)
#define FEATURE_BITS_V2               (1<<1)
#define FEATURE_BITS_V3               (1<<2)
#define FEATURE_BITS_V4               (1<<3)

#define FEATURE_BITS_ADS              (1<<4)
#define FEATURE_BITS_ECMC             (1<<5)
#define FEATURE_BITS_SIM              (1<<6)

  ethercatmcController(const char *portName, const char *ethercatmcPortName,
                       int numAxes, double movingPollPeriod,
                       double idlePollPeriod,
                       const char *optionStr);

  /* Note: the motor/master version does not have it, so we need it here */
  asynStatus writeOctet(asynUser *pasynUser, const char *value,
                        size_t nChars, size_t *nActual);
  void report(FILE *fp, int level);
  asynStatus setMCUErrMsg(const char *value);
  asynStatus configController(int needOk, const char *value);
  asynStatus writeReadOnErrorDisconnect(void);
  ethercatmcAxis* getAxis(asynUser *pasynUser);
  ethercatmcAxis* getAxis(int axisNo);
  int features_;

  protected:
  epicsMutexId      lockADSsocket_;
  void udateMotorLimitsRO(int axisNo);
  void udateMotorLimitsRO(int axisNo, int enabledHighAndLow,
                          double fValueHigh, double fValueLow);
  void handleStatusChange(asynStatus status);
  asynStatus writeReadBinaryOnErrorDisconnect(asynUser *pasynUser,
                                              const char *outdata,
                                              size_t outlen,
                                              char *indata, size_t inlen,
                                              size_t *pnread);

  /* memory bytes via ADS */
  asynStatus writeWriteReadAds(asynUser *pasynUser,
                               AmsHdrType *amsHdr_p, size_t outlen,
                               uint32_t invokeID,
                               uint32_t ads_cmdID,
                               void *indata, size_t inlen,
                               size_t *pnread);
  asynStatus getPlcMemoryViaADS(unsigned indexOffset,
                                void *data, size_t lenInPlc);
  asynStatus setPlcMemoryViaADS(unsigned indexOffset,
                                const void *data, size_t lenInPlc);
  asynStatus getSymbolInfoViaADS(const char *symbolName,
                                 void *data,
                                 size_t lenInPlc);
  asynStatus getSymbolHandleByNameViaADS(const char *symbolName,
                                         uint32_t *handle);

  /* Indexer */
  asynStatus readDeviceIndexer(unsigned devNum, unsigned infoType);
  void parameterFloatReadBack(unsigned axisNo,
                              int initial,
                              unsigned paramIndex,
                              double fValue);
  asynStatus indexerReadAxisParameters(ethercatmcIndexerAxis *pAxis,
                                       unsigned devNum,
                                       unsigned iOffset,
                                       unsigned lenInPlcPara);
  asynStatus poll(void);
  asynStatus newIndexerAxis(ethercatmcIndexerAxis *pAxis,
                            unsigned devNum,
                            unsigned iAllFlags,
                            double   fAbsMin,
                            double   fAbsMax,
                            unsigned iOffset);
  asynStatus updateCfgValue(int axisNo_, int function,
                            double newValue, const char *name);
  asynStatus updateCfgValue(int axisNo_, int function,
                            int newValue, const char *name);
  asynStatus getFeatures(int *pRet);
  asynStatus initialPollIndexer(void);
  asynStatus pollIndexer(void);
  asynStatus writeReadControllerPrint(int traceMask);
  asynStatus writeReadACK(int traceMask);
  asynStatus getPlcMemoryUint(unsigned indexOffset,
                              unsigned *value, size_t lenInPlc);
  asynStatus setPlcMemoryInteger(unsigned indexOffset,
                                 int value, size_t lenInPlc);
  asynStatus getPlcMemoryDouble(unsigned indexOffset,
                                double *value, size_t lenInPlc);
  asynStatus setPlcMemoryDouble(unsigned indexOffset,
                                double value, size_t lenInPlc);

  asynStatus indexerWaitSpecialDeviceIdle(unsigned indexOffset);
  asynStatus indexerParamWaitNotBusy(unsigned indexOffset);
  asynStatus indexerParamRead(int axisNo,
                              unsigned paramIfOffset,
                              unsigned paramIndex,
                              unsigned lenInPlcPara,
                              double *value);
  asynStatus indexerParamWrite(int axisNo,
                               unsigned paramIfOffset,
                               unsigned paramIndex,
                               unsigned lenInPlcPara,
                               double value);

  asynStatus getPlcMemoryFromProcessImage(unsigned indexOffset,
                                          void *data, size_t lenInPlc);
  void addPilsAsynDevInfo(int      axisNo,
                          unsigned indexOffset,
                          unsigned iTypCode,
                          const char *paramName);

  pilsAsynDevInfo_type *findIndexerOutputDevice(int axisNo,
                                                int function,
                                                asynParamType myEPICSParamType);

  struct {
    uint8_t      *pIndexerProcessImage;
    asynStatus   oldStatus;
    unsigned int cntADSstatus;
    unsigned int local_no_ASYN_;
    unsigned int hasConfigError;
    unsigned int initialPollDone;
    unsigned int indexerOffset;
    unsigned int firstDeviceStartOffset;
    unsigned int lastDeviceEndOffset;
    unsigned int specialDbgStrToMcuDeviceLength;
    unsigned int specialDbgStrToMcuDeviceOffset;

    AmsNetidAndPortType remote;
    AmsNetidAndPortType local;
    unsigned adsport;
    int useADSbinary;
    struct {
      unsigned int stAxisStatus_V1  :1;
      unsigned int stAxisStatus_V2  :1;
      unsigned int bSIM             :1;
      unsigned int bECMC            :1;
      unsigned int bADS             :1;
    } supported;
    pilsAsynDevInfo_type pilsAsynDevInfo[50]; /* TODO: dynamic allocation */
    unsigned numPilsAsynDevInfo;
  } ctrlLocal;


  /* First parameter */
  int ethercatmcErr_;
  int ethercatmcStatusCode_;
  int ethercatmcStatusBits_;
  int ethercatmcNamAux0_;
  int ethercatmcNamAux1_;
  int ethercatmcNamAux2_;
  int ethercatmcNamAux3_;
  int ethercatmcNamAux4_;
  int ethercatmcNamAux5_;
  int ethercatmcNamAux6_;
  int ethercatmcNamAux7_;
  int ethercatmcNamBit24_;
  int ethercatmcNamBit25_;
  int ethercatmcFoffVis_;
  int ethercatmcHomProc_RB_;
  int ethercatmcHomPos_RB_;
  int ethercatmcHomProc_;
  int ethercatmcHomPos_;
  int ethercatmcVelToHom_;
  int ethercatmcVelFrmHom_;
  int ethercatmcAccHom_;
  int ethercatmcEncAct_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif

  /* Add parameters here */
  int ethercatmcErrRst_;
  int ethercatmcMCUErrMsg_;
  int ethercatmcDbgStrToMcu_;
  int ethercatmcDbgStrToLog_;
  int ethercatmcVelAct_;
  int ethercatmcVel_RB_;
  int ethercatmcAcc_RB_;
  int ethercatmcCfgVELO_;
  int ethercatmcCfgVMAX_;
  int ethercatmcCfgJVEL_;
  int ethercatmcCfgACCS_;
  int ethercatmcCfgSREV_RB_;
  int ethercatmcCfgUREV_RB_;
  int ethercatmcCfgPMIN_RB_;
  int ethercatmcCfgPMAX_RB_;
  int ethercatmcCfgSPDB_RB_;
  int ethercatmcCfgRDBD_RB_;
  int ethercatmcCfgRDBD_Tim_RB_;
  int ethercatmcCfgRDBD_En_RB_;
  int ethercatmcCfgPOSLAG_RB_;
  int ethercatmcCfgPOSLAG_Tim_RB_;
  int ethercatmcCfgPOSLAG_En_RB_;
  int ethercatmcCfgDHLM_RB_;
  int ethercatmcCfgDLLM_RB_;
  int ethercatmcCfgDHLM_En_RB_;
  int ethercatmcCfgDLLM_En_RB_;
  int ethercatmcCfgDHLM_;
  int ethercatmcCfgDLLM_;
  int ethercatmcCfgDHLM_En_;
  int ethercatmcCfgDLLM_En_;
  int ethercatmcCfgDESC_RB_;
  int ethercatmcCfgEGU_RB_;

  int ethercatmcErrId_;
  /* Last parameter */

  #define FIRST_VIRTUAL_PARAM ethercatmcErr_
  #define LAST_VIRTUAL_PARAM ethercatmcErrId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int) (&LAST_VIRTUAL_PARAM - &FIRST_VIRTUAL_PARAM + 1))

  friend class ethercatmcAxis;
  friend class ethercatmcIndexerAxis;
};

#endif
