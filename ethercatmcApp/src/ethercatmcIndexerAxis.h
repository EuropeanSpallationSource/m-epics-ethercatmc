#ifndef ETHERCATMCINDEXERAXIS_H
#define ETHERCATMCINDEXERAXIS_H

#include <stdint.h>

/* Parameter interface */
/* The highest 3 bits are used for the command itself */
#define PARAM_IF_CMD_MASK                          0xE000
#define PARAM_IF_IDX_MASK                          0x1FFF
/* highes bit is ACK: PLC has done something */
#define PARAM_IF_ACK_MASK                          0x8000


#define PARAM_IF_CMD_INVALID                   0x0000
#define PARAM_IF_CMD_DOREAD                        0x2000
#define PARAM_IF_CMD_DOWRITE                       0x4000
#define PARAM_IF_CMD_BUSY                          0x6000
#define PARAM_IF_CMD_DONE                          0x8000
#define PARAM_IF_CMD_ERR_NO_IDX                    0xA000
#define PARAM_IF_CMD_ERR_READONLY                  0xC000
#define PARAM_IF_CMD_ERR_RETRY_LATER               0xE000

/* Some parameters are functions */
#define PARAM_IF_IDX_FIRST_FUNCTION                128
#define PARAM_IF_IDX_FIRST_CUSTOM_PARA             192
#define PARAM_IF_IDX_LAST_CUSTOM_PARA              223

typedef enum
  {
    PILSparamPermNone = 0,
    PILSparamPermRead = 1,
    PILSparamPermWrite = 2
  } PILSparamPermType;

extern "C" {
  int ethercatmcCreateIndexerAxis(const char *ethercatmcName, int axisNo,
                                  int axisFlags, const char *axisOptionsStr);
  const char *paramIfCmdToString(unsigned cmdSubParamIndex);
  typedef struct {
    uint8_t   paramCtrl[2];
    uint8_t   paramValueRaw[8]; /* May be 4 or 8 bytes */
  } paramIf_type;
};

class epicsShareClass ethercatmcIndexerAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ethercatmcIndexerAxis(class ethercatmcController *pC, int axisNo,
                               int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus setPosition(double);

  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus writeCmdRegisster(unsigned idxStatusCode);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);
  asynStatus stop(double acceleration);
  void setIndexerDevNumOffsetTypeCode(unsigned devNum, unsigned iOffset, unsigned iTypCode);
  void setAuxBitsNotHomedMask(unsigned auxBitsNotHomedMask);
  void setAuxBitsEnabledMask(unsigned auxBitsEnabledMask);
  void setAuxBitsLocalModeMask(unsigned auxBitsLocalModeMask);
  void setAuxBitsHomeSwitchMask(unsigned auxBitsHomeSwitchMask);
  void addPollNowParam(uint8_t paramIndex);
  void       setAxisID(unsigned axisID);
  asynStatus setIntegerParamLog(int function, int newValue, const char *name);
  asynStatus poll(bool *moving);
  asynStatus resetAxis(void);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  asynStatus setStringParamDbgStrToMcu(const char *value);
  asynStatus setStringParam(int function, const char *value);

private:
  ethercatmcController *pC_;

  struct {
    const char *externalEncoderStr;
    struct {
      int          old_ErrorId;
      int          old_hasError;
      unsigned int oldStatusDisconnected : 1;
      unsigned int initialPollNeeded :1;
      unsigned idxStatusCode;
    }  dirty;
    double old_paramValue;
    unsigned pollNowIdx;
    unsigned iTypCode;
    unsigned devNum;
    unsigned axisID;
    unsigned iOffset;
    unsigned lenInPlcPara;
    unsigned paramIfOffset;
    unsigned auxBitsNotHomedMask;
    unsigned auxBitsEnabledMask;
    unsigned auxBitsLocalModeMask;
    unsigned auxBitsHomeSwitchMask;
    unsigned old_paramCtrl;
    unsigned old_idxAuxBits;
    unsigned int hasProblem :1;
    uint8_t pollNowParams[128]; /* 0 terminated list of parameters to be polled */
    PILSparamPermType PILSparamPerm[256];
    } drvlocal;

#ifndef motorMessageTextString
  void updateMsgTxtFromDriver(const char *value);
#endif

  friend class ethercatmcController;
};

#endif
