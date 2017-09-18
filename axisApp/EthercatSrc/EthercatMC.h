/*
FILENAME...   EthercatMC.h
*/

#include "asynAxisController.h"
#include "asynAxisAxis.h"

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif

#define EthercatMCErrString                  "MCUErr"
#define EthercatMCErrIdString                "ErrId"
#define EthercatMCProcHomString              "ProcHom"
#define EthercatMCPosHomString               "PosHom"
#define EthercatMCVelToHomString             "VelToHom"
#define EthercatMCVelFrmHomString            "VelFrmHom"
#define EthercatMCAccHomString               "AccHom"
#define EthercatMCDecHomString               "DecHom"
#define EthercatMCErrRstString               "ErrRst"
#define EthercatMCVelActString               "VelAct"
#define EthercatMCVel_RBString               "Vel-RB"
#define EthercatMCAcc_RBString               "Acc-RB"
#define EthercatMCDec_RBString               "Dec-RB"
#define EthercatMCCFGVELO_String             "CFGVELO"
#define EthercatMCCFGVMAX_String             "CFGVMAX"
#define EthercatMCCFGJVEL_String             "CFGJVEL"
#define EthercatMCCFGJAR_String              "CFGJAR"
#define EthercatMCCHLMString                 "CHLM"
#define EthercatMCCLLMString                 "CLLM"
#define EthercatMCCHLM_EnString              "CHLM-En"
#define EthercatMCCLLM_EnString              "CLLM-En"
#define EthercatMCEnString                   "En"
#define EthercatMCScalSREV_RBString          "SCALSREV-RB"
#define EthercatMCScalUREV_RBString          "SCALUREV-RB"
#define EthercatMCScalRefVelo_RBString       "REFVELO-RB"
#define EthercatMCScalMDIR_RBString          "SCALMDIR-RB"
#define EthercatMCScalEDIR_RBString          "SCALEDIR-RB"
#define EthercatMCScalRDBD_RBString          "SCALRDBD-RB"
#define EthercatMCScalRDBD_Tim_RBString      "SCALRDBD-Tim-RB"
#define EthercatMCScalRDBD_En_RBString       "SCALRDBD-En-RB"


#define EthercatMCMCUErrMsgString            "MCUErrMsg"
#define EthercatMCDbgStrToMcuString          "StrToMCU"

extern "C" {
  int EthercatMCCreateAxis(const char *EthercatMCName, int axisNo,
                      int axisFlags, const char *axisOptionsStr);
  asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                          const char *outdata, size_t outlen,
                                          char *indata, size_t inlen);
  asynStatus checkACK(const char *outdata, size_t outlen, const char *indata);
}

typedef struct {
  /* V1 members */
  int bEnable;           /*  1 */
  int bReset;            /*  2 */
  int bExecute;          /*  3 */
  int nCommand;          /*  4 */
  int nCmdData;          /*  5 */
  double fVelocity;      /*  6 */
  double fPosition;      /*  7 */
  double fAcceleration;  /*  8 */
  double fDecceleration; /*  9 */
  int bJogFwd;           /* 10 */
  int bJogBwd;           /* 11 */
  int bLimitFwd;         /* 12 */
  int bLimitBwd;         /* 13 */
  double fOverride;      /* 14 */
  int bHomeSensor;       /* 15 */
  int bEnabled;          /* 16 */
  int bError;            /* 17 */
  int nErrorId;          /* 18 */
  double fActVelocity;   /* 19 */
  double fActPosition;   /* 20 */
  double fActDiff;       /* 21 */
  int bHomed;            /* 22 */
  int bBusy;             /* 23 */
  /* V2 members */
  double positionRaw;
  int atTarget;
  /* neither V1 nor V2, but calculated here */
  int mvnNRdyNex; /* Not in struct. Calculated in poll() */
  int motorStatusDirection; /* Not in struct. Calculated in pollAll() */
  int motorDiffPostion;     /* Not in struct. Calculated in poll() */
} st_axis_status_type;

class epicsShareClass EthercatMCAxis : public asynAxisAxis
{
public:
  /* These are the methods we override from the base class */
  EthercatMCAxis(class EthercatMCController *pC, int axisNo,
            int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus setPosition(double);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  void       callParamCallbacksUpdateError();
  asynStatus pollAll(bool *moving);
  asynStatus pollAll(bool *moving, st_axis_status_type *pst_axis_status);
  asynStatus poll(bool *moving);

private:
  typedef enum
  {
    eeAxisErrorNoError,
    eeAxisErrorMCUError,
    eeAxisErrorIOCcomError,
    eeAxisErrorIOCcfgError,
    eeAxisErrorCmdError

  } eeAxisErrorType;
  EthercatMCController *pC_;          /**< Pointer to the asynAxisController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    st_axis_status_type old_st_axis_status;
    double stepSize;
    double cfgStepSize;
    const char *externalEncoderStr;
    const char *cfgfileStr;
    const char *cfgDebug_str;
    int axisFlags;
    int MCU_nErrorId;     /* nErrorID from MCU */
    int old_MCU_nErrorId; /* old nErrorID from MCU */
    int old_EPICS_nErrorId; /* old nErrorID from MCU */

    int old_bError;   /* copy of bError */
    unsigned int waitNumPollsBeforeReady;
    int mustStop;
    int nCommand;
    int homed;
    eeAxisErrorType old_eeAxisError;
    eeAxisErrorType eeAxisError;
    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      int          nMotionAxisID;     /* Needed for ADR commands */
      unsigned int stAxisStatus_Vxx :1;
      unsigned int oldStatusDisconnected : 1;
      unsigned int initialUpdate    :1;
      unsigned int sErrorMessage    :1; /* From MCU */
      unsigned int readConfigFile   :1;
      unsigned int features         :1;
    }  dirty;

    struct {
      unsigned int stAxisStatus_V1  :1;
      unsigned int stAxisStatus_V2  :1;
      unsigned int bV1BusyNewStyle  :1;
      unsigned int bSIM             :1;
      unsigned int bECMC            :1;
      unsigned int bADS             :1;
      int          statusVer;           /* 0==V1, busy old style 1==V1, new style*/
                                        /* 2==V2 */
    }  supported;
    /* Error texts when we talk to the controller, there is not an "OK"
       Or, failure in setValueOnAxisVerify() */
    char cmdErrorMessage[80];
    char adsport_str[15]; /* "ADSPORT=12345/" */ /* 14 should be enough, */
    char adsport_zero[1]; /* 15 + 1 for '\' keep us aligned in memory */

  } drvlocal;

  asynStatus handleDisconnect(void);
  asynStatus handleConnect(void);
  asynStatus readConfigFile(void);
  void readBackHighSoftLimit(void);
  void readBackLowSoftLimit(void);
  void readBackConfig(void);
  asynStatus initialUpdate(void);

  asynStatus handleStatusChange(asynStatus status);

  asynStatus writeReadACK(void);
  asynStatus setValueOnAxis(const char* var, int value);
  asynStatus setValueOnAxisVerify(const char *var, const char *rbvar,
                                  int value, unsigned int retryCount);
  asynStatus setValueOnAxis(const char* var, double value);
  asynStatus setValuesOnAxis(const char* var1, double value1, const char* var2, double value2);
  int getMotionAxisID(void);
  void getFeatures(void);
  asynStatus setSAFValueOnAxis(unsigned indexGroup,
                               unsigned indexOffset,
                               int value);

  asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
                                     unsigned indexOffset,
                                     int value,
                                     unsigned int retryCount);

  asynStatus setSAFValueOnAxis(unsigned indexGroup,
                               unsigned indexOffset,
                               double value);

  asynStatus setSAFValueOnAxisVerify(unsigned indexGroup,
                                     unsigned indexOffset,
                                     double value,
                                     unsigned int retryCount);

  asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
                                      unsigned indexOffset,
                                      const char *name,
                                      int *value);

  asynStatus getSAFValuesFromAxisPrint(unsigned iIndexGroup,
                                       unsigned iIndexOffset,
                                       const char *iname,
                                       int *iValue,
                                       unsigned fIndexGroup,
                                       unsigned fIndexOffset,
                                       const char *fname,
                                       double *fValue);

  asynStatus getSAFValueFromAxisPrint(unsigned indexGroup,
                                      unsigned indexOffset,
                                      const char *name,
                                      double *value);

  asynStatus getValueFromAxis(const char* var, int *value);
  asynStatus getValueFromAxis(const char* var, double *value);
  asynStatus getStringFromAxis(const char* var, char *value, size_t maxlen);
  asynStatus getValueFromController(const char* var, double *value);

  asynStatus resetAxis(void);
  asynStatus enableAmplifier(int);
  asynStatus sendVelocityAndAccelExecute(double maxVelocity, double acceleration_time);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  asynStatus setStringParamDbgStrToMcu(const char *value);
  asynStatus setStringParam(int function, const char *value);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);

  friend class EthercatMCController;
};

class epicsShareClass EthercatMCController : public asynAxisController {
public:
  EthercatMCController(const char *portName, const char *EthercatMCPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  asynStatus setMCUErrMsg(const char *value);
  asynStatus configController(int needOk, const char *value);
  asynStatus writeReadOnErrorDisconnect(void);
  EthercatMCAxis* getAxis(asynUser *pasynUser);
  EthercatMCAxis* getAxis(int axisNo);
  protected:
  void handleStatusChange(asynStatus status);
  struct {
    unsigned int local_no_ASYN_;
    unsigned int hasConfigError;
    unsigned int isConnected;
  } ctrlLocal;

  /* First parameter */
  int EthercatMCErr_;
  int EthercatMCProcHom_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif

  /* Add parameters here */
  int EthercatMCErrRst_;
  int EthercatMCMCUErrMsg_;
  int EthercatMCDbgStrToMcu_;
  int EthercatMCPosHom_;
  int EthercatMCVelToHom_;
  int EthercatMCVelFrmHom_;
  int EthercatMCAccHom_;
  int EthercatMCDecHom_;
  int EthercatMCVelAct_;
  int EthercatMCVel_RB_;
  int EthercatMCAcc_RB_;
  int EthercatMCDec_RB_;
  int EthercatMCCFGVELO_;
  int EthercatMCCFGVMAX_;
  int EthercatMCCFGJVEL_;
  int EthercatMCCFGJAR_;
  int EthercatMCScalSREV_RB_;
  int EthercatMCScalUREV_RB_;
  int EthercatMCScalRefVelo_RB_;
  int EthercatMCScalMDIR_RB_;
  int EthercatMCScalEDIR_RB_;
  int EthercatMCScalRDBD_RB_;
  int EthercatMCScalRDBD_Tim_RB_;
  int EthercatMCScalRDBD_En_RB_;
  int EthercatMCCHLM_;
  int EthercatMCCLLM_;
  int EthercatMCCHLM_En_;
  int EthercatMCCLLM_En_;
  int EthercatMCEn_;
  int EthercatMCErrId_;
  /* Last parameter */

  #define FIRST_VIRTUAL_PARAM EthercatMCErr_
  #define LAST_VIRTUAL_PARAM EthercatMCErrId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int) (&LAST_VIRTUAL_PARAM - &FIRST_VIRTUAL_PARAM + 1))

  friend class EthercatMCAxis;
};
