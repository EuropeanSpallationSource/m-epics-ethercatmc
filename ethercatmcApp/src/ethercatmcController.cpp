/*
FILENAME... ethercatmcController.cpp
*/

#include "ethercatmcController.h"

#include <asynOctetSyncIO.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsVersion.h>
#include <iocsh.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ethercatmcIndexerAxis.h"

#ifdef ETHERCATMC_TCBSD
#define POSIX
#include "/usr/local/include/TcAdsAPI.h"
#include "/usr/local/include/TcAdsDef.h"
#endif

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO 0x0040
#endif

#ifndef EPICS_VERSION_INT
#define EPICS_VERSION_INT                                        \
  VERSION_INT(EPICS_VERSION, EPICS_REVISION, EPICS_MODIFICATION, \
              EPICS_PATCH_LEVEL)
#endif

#if EPICS_VERSION_INT < VERSION_INT(7, 0, 0, 0)
epicsShareExtern volatile int interruptAccept;
#else
#include <dbCoreAPI.h>
DBCORE_API extern volatile int interruptAccept;
#endif

const char *modNamEMC = "ethercatmc:: ";

const static char *const strethercatmcCreateController =
    "ethercatmcCreateController";
const static char *const strethercatmcCreateIndexerAxisDef =
    "ethercatmcCreateIndexerAxis";
const static char *const strethercatmcCreateAsynParamDef =
    "ethercatmcCreateAsynParam";
const static char *const strethercatmcStartPollerDef = "ethercatmcStartPoller";

const static char *const modulName = "ethercatmc::";

extern "C" double ethercatmcgetNowTimeSecs(void) {
  epicsTimeStamp nowTime;
  epicsTimeGetCurrent(&nowTime);
  return nowTime.secPastEpoch + (nowTime.nsec * 0.000000001);
}

extern "C" const char *errStringFromErrId(int nErrorId) {
  switch (nErrorId) {
    case 0x0707:
      return "Dev notrdyST";
    case 0x4221:
      return "Velo illegal";
    case 0x4223:
      return "Axis pos en";
    case 0x4225:
      return "Drv not en";
    case 0x4257:
      return "Fn not allwd";
    case 0x4450:
      return "Enc undeflw";
    case 0x4451:
      return "Enc overflw";
    case 0x4260:
      return "Amplifie off";
    case 0x4263:
      return "Still proce";
    case 0x42A0:
      return "Consequ Err";
    case 0x42EF:
      return "Locked Stop";
    case 0x4359:
      return "Velo unaccep";
    case 0x4460:
      return "Low soft lim";
    case 0x4461:
      return "High softlim";
    case 0x4462:
      return "Min position";
    case 0x4463:
      return "Max position";
    case 0x4464:
      return "Enc HW error";
    case 0x4466:
      return "Enc IO data";
    case 0x4467:
      return "Enc inv pos";
    case 0x4550:
      return "Follw errpos";
    case 0x4551:
      return "Follw errvel";
    case 0x4650:
      return "DrvHW notrdy";
    case 0x4655:
      return "Inv IO data";
    case 0x4B07:
      return "Timo axis FB";
    case 0x4B09:
      return "Axis not rdy";
    case 0x4B0A:
      return "Homing faild";
    case 0x10001:
      return "Extract Timeout";
    case 0x10002:
      return "Retract Timeout";
    case 0x10003:
      return "Not Moving Extract";
    case 0x10004:
      return "Not Moving Retract";
    case 0x10005:
      return "No PSS Permit";
    case 0x10006:
      return "Retract Interlocked";
    case 0x10007:
      return "Extract Interlocked";
    case 0x10008:
      return "Air Pressure Low";
    case 0x10009:
      return "Air Pressure High";
    case 0x1000A:
      return "NoSignalEndSwitchBwd";
    case 0x1000B:
      return "NoSignalEndSwitchFwd";
    default:
      return "";
  }
}
extern "C" const char *stringFromAsynParamType(asynParamType paramType) {
  switch (paramType) {
    case asynParamNotDefined:
      return "asynParamNotDefined";
    case asynParamInt32:
      return "asynParamInt32";
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
    case asynParamInt64:
      return "asynParamInt64";
#endif
    case asynParamUInt32Digital:
      return "asynParamUInt32Digital";
    case asynParamFloat64:
      return "asynParamFloat64";
    case asynParamOctet:
      return "asynParamOctet";
    case asynParamInt8Array:
      return "asynParamInt8Array";
    case asynParamInt16Array:
      return "asynParamInt16Array";
    case asynParamInt32Array:
      return "asynParamInt32Array";
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
    case asynParamInt64Array:
      return "asynParamInt64Array";
#endif
    case asynParamFloat32Array:
      return "asynParamFloat32Array";
    case asynParamFloat64Array:
      return "asynParamFloat64Array";
    case asynParamGenericPointer:
      return "asynParamGenericPointer";
    default:
      return "asynParamXXXXX";
  }
}

extern "C" const char *ethercatmcstrStatus(asynStatus status) {
  switch ((int)status) {
    case asynSuccess:
      return "asynSuccess";
    case asynTimeout:
      return "asynTimeout";
    case asynOverflow:
      return "asynOverflow";
    case asynError:
      return "asynError";
    case asynDisconnected:
      return "asynDisconnected";
    case asynDisabled:
      return "asynDisabled";
    case asynParamAlreadyExists:
      return "asynParamAlreadyExists";
    case asynParamNotFound:
      return "asynParamNotFound";
    case asynParamWrongType:
      return "asynParamWrongType";
    case asynParamBadIndex:
      return "asynParamBadIndex";
    case asynParamUndefined:
      return "asynParamUndefined";
    default:
      return "??";
  }
}

/** Creates a new ethercatmcController object.
 * \param[in] portName          The name of the asyn port that will be created
 * for this driver \param[in] MotorPortName     The name of the
 * drvAsynSerialPort that was created previously to connect to the ethercatmc
 * controller \param[in] numAxes           The number of axes that this
 * controller supports \param[in] movingPollPeriod  The time between polls when
 * any axis is moving \param[in] idlePollPeriod    The time between polls when
 * no axis is moving
 */

#if ETHERCATMC_ASYN_VERSION_INT < VERSION_INT_4_32
#error Need_asyn_later_than_4_32
#endif

ethercatmcController::ethercatmcController(const char *portName,
                                           const char *MotorPortName,
                                           int numAxes, double movingPollPeriod,
                                           double idlePollPeriod,
                                           const char *optionStr)
    : asynMotorController(
          portName, numAxes,
          0,  // Olbsolete: Fixed number of additional asyn parameters
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
          asynInt64Mask |
#endif
              asynParamMetaMask | asynUInt32DigitalMask | asynEnumMask,
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
          asynInt64Mask |
#endif
              asynParamMetaMask | asynUInt32DigitalMask | asynEnumMask,
          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
          1,     // autoconnect
          0, 0)  // Default priority and stack size
{
  asynStatus status;

  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  memset(&defAsynPara, 0, sizeof(defAsynPara));
  ctrlLocal.oldStatus = asynError;  // asynDisconnected;
  ctrlLocal.cntADSstatus = 0;
  /* Outputs */
  createParam(ethercatmcDbgStrToMcuString, asynParamOctet,
              &defAsynPara.ethercatmcDbgStrToMcu_);
  createParam(ethercatmcDbgStrToLogString, asynParamOctet,
              &defAsynPara.ethercatmcDbgStrToLog_);
  createParam(pilsLongoutRecordString, asynParamInt32,
              &defAsynPara.pilsLongoutRecord_);
  createParam(pilsBoMinMaxString, asynParamInt32, &defAsynPara.pilsBoMinMax_);
  createParam(ethercatmcErrRstString, asynParamInt32,
              &defAsynPara.ethercatmcErrRst_);

  /* inputs: do not change too much without updating
   * setAlarmStatusSeverityAllReadbacks() */
  createParam(ethercatmcMcuErrString, asynParamInt32,
              &defAsynPara.ethercatmcMcuErr_);
  createParam(ethercatmcErrIdString, asynParamInt32,
              &defAsynPara.ethercatmcErrId_);
  createParam(ethercatmcErrTxtString, asynParamOctet,
              &defAsynPara.ethercatmcErrTxt_);

  createParam(ethercatmcRawEncStepString, asynParamInt32,
              &defAsynPara.ethercatmcRawEncStep_);
  createParam(ethercatmcRawMtrStepString, asynParamInt32,
              &defAsynPara.ethercatmcRawMtrStep_);
  createParam(ethercatmcRawMtrVeloString, asynParamInt32,
              &defAsynPara.ethercatmcRawMtrVelo_);
  createParam(ethercatmcStatusCodeString, asynParamInt32,
              &defAsynPara.ethercatmcStatusCode_);
  createParam(ethercatmcStatusBitsString, asynParamUInt32Digital,
              &defAsynPara.ethercatmcStatusBits_);
  createParam(ethercatmcFoffVisString, asynParamInt32,
              &defAsynPara.ethercatmcFoffVis_);
  createParam(ethercatmcHomeVisString, asynParamInt32,
              &defAsynPara.ethercatmcHomeVis_);
  createParam(ethercatmcHomProc_RBString, asynParamInt32,
              &defAsynPara.ethercatmcHomProc_RB_);
  createParam(ethercatmcHomPos_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcHomPos_RB_);
  createParam(ethercatmcVelActString, asynParamFloat64,
              &defAsynPara.ethercatmcVelAct_);
  createParam(ethercatmcVel_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcVel_RB_);
  createParam(ethercatmcAcc_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcAcc_RB_);
  createParam(ethercatmcCfgIdleCurrent_String, asynParamFloat64,
              &defAsynPara.ethercatmcCfgIdleCurrent_);
  createParam(ethercatmcCfgMoveCurrent_String, asynParamFloat64,
              &defAsynPara.ethercatmcCfgMoveCurrent_);
  createParam(ethercatmcPTPdiffTimeIOC_MCUString, asynParamFloat64,
              &defAsynPara.ethercatmcPTPdiffTimeIOC_MCU_);
  createParam(ethercatmcPTPdiffXYtime_MCUString, asynParamFloat64,
              &defAsynPara.ethercatmcPTPdiffXYtime_MCU_);
  createParam(ethercatmcPTPallGoodString, asynParamInt32,
              &defAsynPara.ethercatmcPTPallGood_);
  createParam(ethercatmcRBV_TSEString, asynParamFloat64,
              &defAsynPara.ethercatmcRBV_TSE_);
  createParam(pilsLonginActualString, asynParamInt32,
              &defAsynPara.pilsLonginActual_);
  createParam(pilsLonginTargetString, asynParamInt32,
              &defAsynPara.pilsLonginTarget_);
  createParam(ethercatmcAuxBits07_String, asynParamInt32,
              &defAsynPara.ethercatmcAuxBits07_);
  createParam(ethercatmcNamAux0_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux0_);
  createParam(ethercatmcNamAux1_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux1_);
  createParam(ethercatmcNamAux2_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux2_);
  createParam(ethercatmcNamAux3_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux3_);
  createParam(ethercatmcNamAux4_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux4_);
  createParam(ethercatmcNamAux5_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux5_);
  createParam(ethercatmcNamAux6_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux6_);
  createParam(ethercatmcNamAux7_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux7_);
  createParam(ethercatmcNamAux8_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux8_);
  createParam(ethercatmcNamAux9_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux9_);
  createParam(ethercatmcNamAux10_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux10_);
  createParam(ethercatmcNamAux11_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux11_);
  createParam(ethercatmcNamAux12_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux12_);
  createParam(ethercatmcNamAux13_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux13_);
  createParam(ethercatmcNamAux14_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux14_);
  createParam(ethercatmcNamAux15_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux15_);
  createParam(ethercatmcNamAux16_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux16_);
  createParam(ethercatmcNamAux17_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux17_);
  createParam(ethercatmcNamAux18_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux18_);
  createParam(ethercatmcNamAux19_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux19_);
  createParam(ethercatmcNamAux20_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux20_);
  createParam(ethercatmcNamAux21_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux21_);
  createParam(ethercatmcNamAux22_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux22_);
  createParam(ethercatmcNamAux23_String, asynParamOctet,
              &defAsynPara.ethercatmcNamAux23_);
  createParam(ethercatmcPollScalingString, asynParamInt32,
              &defAsynPara.ethercatmcPollScaling_);
  createParam(ethercatmcCfgVELO_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgVELO_RB_);
  createParam(ethercatmcCfgVMAX_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgVMAX_RB_);
  createParam(ethercatmcCfgJVEL_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgJVEL_RB_);
  createParam(ethercatmcCfgHVEL_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgHVEL_RB_);
  createParam(ethercatmcCfgACCS_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgACCS_RB_);
  createParam(ethercatmcCfgDHLMRBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgDHLM_RB_);
  createParam(ethercatmcCfgDLLMRBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgDLLM_RB_);
  createParam(ethercatmcCfgDHLMString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgDHLM_);
  createParam(ethercatmcCfgDLLMString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgDLLM_);
  createParam(ethercatmcCfgDHLM_EnString, asynParamInt32,
              &defAsynPara.ethercatmcCfgDHLM_En_);
  createParam(ethercatmcCfgDLLM_EnString, asynParamInt32,
              &defAsynPara.ethercatmcCfgDLLM_En_);

  createParam(ethercatmcCfgSREV_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgSREV_RB_);
  createParam(ethercatmcCfgUREV_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgUREV_RB_);
  createParam(ethercatmcCfgPMIN_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgPMIN_RB_);
  createParam(ethercatmcCfgPMAX_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgPMAX_RB_);
  createParam(ethercatmcCfgSPDB_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgSPDB_RB_);
  createParam(ethercatmcCfgRDBD_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgRDBD_RB_);
  createParam(ethercatmcCfgRDBD_Tim_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgRDBD_Tim_RB_);
  createParam(ethercatmcCfgRDBD_En_RBString, asynParamInt32,
              &defAsynPara.ethercatmcCfgRDBD_En_RB_);
  createParam(ethercatmcCfgPOSLAG_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgPOSLAG_RB_);
  createParam(ethercatmcCfgPOSLAG_Tim_RBString, asynParamFloat64,
              &defAsynPara.ethercatmcCfgPOSLAG_Tim_RB_);
  createParam(ethercatmcCfgPOSLAG_En_RBString, asynParamInt32,
              &defAsynPara.ethercatmcCfgPOSLAG_En_RB_);

  createParam(ethercatmcCfgDESC_RBString, asynParamOctet,
              &defAsynPara.ethercatmcCfgDESC_RB_);
  createParam(ethercatmcCfgEGU_RBString, asynParamOctet,
              &defAsynPara.ethercatmcCfgEGU_RB_);
  /* No more to be included in setAlarmStatusSeverityAllReadbacks() */

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x)
   */
  createParam(motorRecResolutionString, asynParamFloat64, &motorRecResolution_);
  createParam(motorRecDirectionString, asynParamInt32, &motorRecDirection_);
  createParam(motorRecOffsetString, asynParamFloat64, &motorRecOffset_);
#endif

  /* Connect to ethercatmc controller */
  status =
      pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s cannot connect to motor controller\n", modulName);
  }
  printf("%s:%d %s optionStr=\"%s\"\n", __FILE__, __LINE__, modulName,
         optionStr ? optionStr : "NULL");
  if (optionStr && optionStr[0]) {
    const char *const adsPort_str = "adsPort=";
    const char *const amsNetIdRemote_str = "amsNetIdRemote=";
    const char *const amsNetIdLocal_str = "amsNetIdLocal=";
    const char *const ipaddr_str = "ipaddr=";
    // const char * const ipport_str = "ipport=";
    char *pOptions = strdup(optionStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;
    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, adsPort_str, strlen(adsPort_str))) {
        pThisOption += strlen(adsPort_str);
        int adsPort = atoi(pThisOption);
        if (adsPort > 0) {
          ctrlLocal.adsport = (unsigned)adsPort;
        }
      } else if (!strncmp(pThisOption, amsNetIdRemote_str,
                          strlen(amsNetIdRemote_str))) {
        AmsNetidAndPortType ams_netid_port;
        unsigned amsRemotePort = ctrlLocal.adsport;
        int nvals;
        memset(&ams_netid_port, 0, sizeof(ams_netid_port));
        pThisOption += strlen(amsNetIdRemote_str);
        nvals = sscanf(pThisOption, "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu:%u",
                       &ams_netid_port.netID[0], &ams_netid_port.netID[1],
                       &ams_netid_port.netID[2], &ams_netid_port.netID[3],
                       &ams_netid_port.netID[4], &ams_netid_port.netID[5],
                       &amsRemotePort);
        if (nvals == 7) {
          /* overwrite the adsPort= statement */
          ctrlLocal.adsport = amsRemotePort;
        }
        if (nvals >= 6) {
          ams_netid_port.port_low = (uint8_t)ctrlLocal.adsport;
          ams_netid_port.port_high = (uint8_t)(ctrlLocal.adsport >> 8);
          memcpy(&ctrlLocal.remote, &ams_netid_port, sizeof(ctrlLocal.remote));
        }
        printf("%s:%d %s amsNetIdRemote=%u.%u.%u.%u.%u.%u:%u\n", __FILE__,
               __LINE__, modulName, ams_netid_port.netID[0],
               ams_netid_port.netID[1], ams_netid_port.netID[2],
               ams_netid_port.netID[3], ams_netid_port.netID[4],
               ams_netid_port.netID[5], ctrlLocal.adsport);
      } else if (!strncmp(pThisOption, amsNetIdLocal_str,
                          strlen(amsNetIdLocal_str))) {
        AmsNetidAndPortType ams_netid_port;
        int nvals;
        memset(&ams_netid_port, 0, sizeof(ams_netid_port));
        pThisOption += strlen(amsNetIdLocal_str);
        nvals = sscanf(pThisOption, "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &ams_netid_port.netID[0], &ams_netid_port.netID[1],
                       &ams_netid_port.netID[2], &ams_netid_port.netID[3],
                       &ams_netid_port.netID[4], &ams_netid_port.netID[5]);
        printf("%s:%d %s amsNetIdLocal=%u.%u.%u.%u.%u.%u:%u\n", __FILE__,
               __LINE__, modulName, ams_netid_port.netID[0],
               ams_netid_port.netID[1], ams_netid_port.netID[2],
               ams_netid_port.netID[3], ams_netid_port.netID[4],
               ams_netid_port.netID[5], ctrlLocal.adsport);
        if (nvals == 6) {
          ams_netid_port.port_low = (uint8_t)ctrlLocal.adsport;
          ams_netid_port.port_high = (uint8_t)(ctrlLocal.adsport >> 8);
          memcpy(&ctrlLocal.local, &ams_netid_port, sizeof(ctrlLocal.local));
        }
      } else if (!strncmp(pThisOption, ipaddr_str, strlen(ipaddr_str))) {
        char buf[128];
        pThisOption += strlen(ipaddr_str);
        snprintf(buf, sizeof(buf), "Connecting %s", pThisOption);
#ifdef motorMessageTextString
        (void)setStringParam(motorMessageText_, buf);
#endif
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_INFO, "%s optionStr=\"%s\"\n",
            modulName, optionStr ? optionStr : "NULL");

  setAlarmStatusSeverityAllReadbacks(asynDisconnected);
  if (movingPollPeriod && idlePollPeriod) {
    /*  Find additional devices/asynParams */
    poll();
  }
  if (movingPollPeriod && idlePollPeriod) {
    startPoller(movingPollPeriod, idlePollPeriod, 2);
  }
}

ethercatmcController::~ethercatmcController() {
#ifdef ETHERCATMC_TCBSD
  asynUser *pasynUser = pasynUserController_;
  if (ctrlLocal.tcbsdLocalPort) {
    ads_i32 ret = AdsPortClose();
    asynPrint(pasynUser, ASYN_TRACE_INFO,
              "~ethercatmcController: AdsPortClose ret=%u\n", (unsigned)ret);
    ctrlLocal.tcbsdLocalPort = 0;
  }
#endif
}

/** Creates a new ethercatmcController object.
 * Configuration command, called directly or from iocsh
 * \param[in] portName          The name of the asyn port that will be created
 * for this driver \param[in] MotorPortName  The name of the drvAsynIPPPort that
 * was created previously to connect to the ethercatmc controller \param[in]
 * numAxes           The number of axes that this controller supports (0 is not
 * used) \param[in] movingPollPeriod  The time in ms between polls when any axis
 * is moving \param[in] idlePollPeriod    The time in ms between polls when no
 * axis is moving
 */
extern "C" int ethercatmcCreateController(const char *portName,
                                          const char *MotorPortName,
                                          int numAxes, int movingPollPeriod,
                                          int idlePollPeriod,
                                          const char *optionStr) {
  new ethercatmcController(portName, MotorPortName, 1 + numAxes,
                           movingPollPeriod / 1000., idlePollPeriod / 1000.,
                           optionStr);
  return (asynSuccess);
}

asynStatus ethercatmcController::ethercatmcCreateParam(
    const char *paramName, asynParamType myEPICSParamType, int *pFunction) {
  asynStatus status;
  int function;
  status = findParam(paramName, &function);
  if (status == asynSuccess) {
    *pFunction = function;
    return status;
  }
  status = createParam(paramName, myEPICSParamType, pFunction);
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s paramName='%s' paramType=%s function=%d status=%s (%d)\n",
            modNamEMC, paramName, stringFromAsynParamType(myEPICSParamType),
            *pFunction, ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus ethercatmcController::ethercatmcStartPoller(double movingPollPeriod,
                                                       double idlePollPeriod) {
  return startPoller(movingPollPeriod / 1000., idlePollPeriod / 1000., 2);
}

extern "C" int ethercatmcCreateAsynParam(const char *ethercatmcName,
                                         const char *paramName,
                                         const char *paramType) {
  ethercatmcController *pC;
  int newFunction = 0;
  asynParamType myEPICSParamType;

  if (!ethercatmcName || !paramName || !paramType) {
    printf(
        "ethercatmcCreateAsynParam MCU1 paramName "
        "[Float64|Int32|Int64|UInt32Digital]\n");
    return asynError;
  }
  pC = (ethercatmcController *)findAsynPortDriver(ethercatmcName);
  if (!pC) {
    printf("Error port %s not found\n", ethercatmcName);
    return asynError;
  }
  if (!strcmp(paramType, "Float64")) {
    myEPICSParamType = asynParamFloat64;
  } else if (!strcmp(paramType, "Int32")) {
    myEPICSParamType = asynParamInt32;
  } else if (!strcmp(paramType, "UInt32Digital")) {
    myEPICSParamType = asynParamUInt32Digital;
  } else if (!strcmp(paramType, "Int64")) {
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
    myEPICSParamType = asynParamInt64;
#else
    myEPICSParamType = asynParamFloat64;
#endif
  } else if (!strcmp(paramType, "Octet")) {
    myEPICSParamType = asynParamOctet;
  } else {
    printf("ethercatmcCreateAsynParam: paramType=%s not supported\n",
           paramType);
    return asynError;
  }
  pC->ethercatmcCreateParam(paramName, myEPICSParamType, &newFunction);
  return asynSuccess;
}

extern "C" int ethercatmcStartPoller_C(const char *ethercatmcName,
                                       int movingPollPeriod,
                                       int idlePollPeriod) {
  ethercatmcController *pC;

  if (!ethercatmcName) {
    printf("ethercatmcStartPoller MCU1\n");
    return asynError;
  }
  pC = (ethercatmcController *)findAsynPortDriver(ethercatmcName);
  if (!pC) {
    printf("ethercatmcStartPoller: Error port %s not found\n", ethercatmcName);
    return asynError;
  }
  return pC->ethercatmcStartPoller((double)movingPollPeriod,
                                   (double)idlePollPeriod);
}

extern "C" asynStatus disconnect_C(asynUser *pasynUser) {
  asynStatus status = asynError;
  asynInterface *pasynInterface = NULL;
  asynCommon *pasynCommon = NULL;
  pasynInterface =
      pasynManager->findInterface(pasynUser, asynCommonType, 0 /* FALSE */);
  if (pasynInterface) {
    pasynCommon = (asynCommon *)pasynInterface->pinterface;
    status = pasynCommon->disconnect(pasynInterface->drvPvt, pasynUser);
    if (status != asynSuccess) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                "%s status=%s (%d)\n", modulName, ethercatmcstrStatus(status),
                (int)status);
    }
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%s pasynInterface=%p pasynCommon=%p\n", modulName,
              pasynInterface, pasynCommon);
  }
  return status;
}

void ethercatmcController::udateMotorLimitsRO(int axisNo) {
  double fValueHigh = 0.0, fValueLow = 0.0;
  int enabledHigh = 0, enabledLow = 0;

  /* When the integer parameter is undefined, 0 is returned,
     same as not enabled */
  getIntegerParam(axisNo, defAsynPara.ethercatmcCfgDHLM_En_, &enabledHigh);
  getIntegerParam(axisNo, defAsynPara.ethercatmcCfgDLLM_En_, &enabledLow);

  if (enabledHigh && enabledLow) {
    asynStatus status1, status2;
    status1 =
        getDoubleParam(axisNo, defAsynPara.ethercatmcCfgDHLM_RB_, &fValueHigh);
    status2 =
        getDoubleParam(axisNo, defAsynPara.ethercatmcCfgDLLM_RB_, &fValueLow);

    if (status1 || status2) {
      udateMotorLimitsRO(axisNo, 0, 0.0, 0.0);
      return;
    }
  }
  udateMotorLimitsRO(axisNo, enabledHigh && enabledLow, fValueHigh, fValueLow);
}

void ethercatmcController::udateMotorLimitsRO(int axisNo, int enabledHighAndLow,
                                              double fValueHigh,
                                              double fValueLow) {
#ifdef motorHighLimitROString
  static const double fABSMIN = -3.0e+38;
  static const double fABSMAX = 3.0e+38;
  int valid = 1;
  if (fValueLow >= fValueHigh || (fValueLow <= fABSMIN) ||
      (fValueHigh >= fABSMAX)) {
    /* Any limit not active or out of range: set everything to 0 */
    valid = 0;
  }

  if (!enabledHighAndLow || !valid) {
    /* Any limit not active or out of range: set everything to 0 */
    fValueHigh = fValueLow = 0.0;
  }
  asynMotorAxis *pAxis = getAxis(axisNo);
  if (pAxis) {
    double oldValueHigh, oldValueLow;
    getDoubleParam(axisNo, motorHighLimitRO_, &oldValueHigh);
    getDoubleParam(axisNo, motorLowLimitRO_, &oldValueLow);
    if ((fValueHigh != oldValueHigh) || (fValueLow != oldValueLow)) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sudateMotorLimitsRO(%d) enabledHighAndLow=%d valid=%d "
                "fValueHigh=%g fValueLow=%g\n",
                modNamEMC, axisNo, enabledHighAndLow, valid, fValueHigh,
                fValueLow);
    }

    /* We need the overload function from asynMotorAxis to
       let the values ripple into the motorRecord */
    pAxis->setDoubleParam(motorHighLimitRO_, fValueHigh);
    pAxis->setDoubleParam(motorLowLimitRO_, fValueLow);
  }
#else
  (void)axisNo;
  (void)enabledHighAndLow;
  (void)fValueHigh;
  (void)fValueLow;
#endif
}

void ethercatmcController::handleStatusChangeFL(asynStatus status,
                                                const char *fileName,
                                                int lineNo) {
  if (status != ctrlLocal.oldStatus) {
    asynPrint(
        pasynUserController_, ASYN_TRACE_INFO,
        "%s %s:%d %s oldStatus=%s(%d) newStatus=%s(%d)\n", modNamEMC, fileName,
        lineNo, "handleStatusChange", ethercatmcstrStatus(ctrlLocal.oldStatus),
        (int)ctrlLocal.oldStatus, ethercatmcstrStatus(status), (int)status);
    if (status) {
      /* Connected -> Disconnected */
      int axisNo;
      ctrlLocal.initialPollDone = 0;
      setAlarmStatusSeverityAllReadbacks(asynDisconnected);
      indexerDisconnected();

      /* Keep bits that are specified via options,
         clear bits that are fetched from the controller */
#ifdef motorMessageTextString
      const char *pErrorMessage = "MCU Disconnected";
      if (pasynUserController_ && pasynUserController_->errorMessage[0]) {
        const char *pLastColon = pasynUserController_->errorMessage;
        while (pLastColon) {
          asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                    "%sDisconnected pErrorMessage=\"%s\" pLastColon=\"%s\"\n",
                    modNamEMC, pErrorMessage ? pErrorMessage : "null",
                    pLastColon ? pLastColon : "null");
          pLastColon = strchr(pLastColon, ':');
          if (pLastColon) {
            pLastColon++;  // Jump over ':'
            pErrorMessage = pLastColon;
          }
        }
        while (pErrorMessage && pErrorMessage[0] == ' ') {
          pErrorMessage++;
        }
      }
      setIntegerParam(motorMessageIsFromDriver_, 1);
      (void)setStringParam(motorMessageText_, pErrorMessage);
#endif
      for (axisNo = 0; axisNo < numAxes_; axisNo++) {
        asynMotorAxis *pAxis = getAxis(axisNo);
        if (!pAxis) continue;
        pAxis->setIntegerParam(motorStatusCommsError_, 1);
        setIntegerParam(axisNo, defAsynPara.ethercatmcFoffVis_, 0);
        setIntegerParam(axisNo, defAsynPara.ethercatmcHomeVis_, 0);
        pAxis->callParamCallbacks();
      }
    } else {
      /* Disconnected -> Connected */
      ctrlLocal.cntADSstatus = 0;
#ifdef motorMessageTextString
      setAlarmStatusSeverityAllAxes(motorMessageText_, asynSuccess);
#endif
    }
    ctrlLocal.oldStatus = status;
  }
  callParamCallbacks();
}

asynStatus ethercatmcController::poll(void) {
  asynStatus status = asynSuccess;

  asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
            "%spoll interruptAccept=%d ctrlLocal.initialPollDone=%d\n",
            modNamEMC, interruptAccept, ctrlLocal.initialPollDone);
  if (!interruptAccept) {
    return asynSuccess;
  }

  ctrlLocal.callBackNeeded = 0;
  if (!ctrlLocal.initialPollDone) {
    status = indexerInitialPoll();
    if (!status) {
      handleStatusChange(status);
      ctrlLocal.initialPollDone = 1;
    } else {
      int i = 1;
      while (i < numAxes_) {
        setIntegerParam(i, motorStatusCommsError_, 1);
        callParamCallbacks(i);
        i++;
      }
    }
  } else {
    status = indexerPoll();
    if (status) {
      handleStatusChange(status);
    }
  }
  for (int axisNo = 0; axisNo < numAxes_; axisNo++) {
    if ((ctrlLocal.callBackNeeded >> axisNo & 1) || (getAxis(axisNo))) {
      callParamCallbacks(axisNo);
    }
  }

  return status;
}

asynStatus ethercatmcController::updateCfgValue(int axisNo_, int function,
                                                double newValue,
                                                const char *name) {
  double oldValue;
  asynStatus status = getDoubleParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) %s=%f\n", modNamEMC, axisNo_, name,
              newValue);
  } else if (newValue != oldValue) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%f new%s=%f\n", modNamEMC, axisNo_,
              name, oldValue, name, newValue);
  }
  setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
  return setDoubleParam(axisNo_, function, newValue);
}

asynStatus ethercatmcController::updateCfgValue(int axisNo_, int function,
                                                int newValue,
                                                const char *name) {
  int oldValue;
  asynStatus status = getIntegerParam(axisNo_, function, &oldValue);
  if (status) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) %s=%d\n", modNamEMC, axisNo_, name,
              newValue);
  } else if (newValue != oldValue) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%d new%s=%d\n", modNamEMC, axisNo_,
              name, oldValue, name, newValue);
  }
  setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
  return setIntegerParam(axisNo_, function, newValue);
}

/** Called when asyn clients call pasynOctetSyncIO->write().
 * Extracts the function and axis number from pasynUser.
 * Sets the value in the parameter library.
 * \param[in] pasynUser asynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 * \param[in] nChars len (but we only support strings ?!).
 * \param[out] nActual. number of octets that had been written */
asynStatus ethercatmcController::writeOctet(asynUser *pasynUser,
                                            const char *value, size_t nChars,
                                            size_t *nActual) {
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  int function = pasynUser->reason;
  pAxis = getAxis(pasynUser);

  status = pAxis->setStringParam(function, value);
  if (status == asynSuccess) *nActual = strlen(value);
  return status;
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information it calls
 * asynMotorController::report()
 */
void ethercatmcController::report(FILE *fp, int level) {
  fprintf(fp,
          "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle "
          "poll period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Code for iocsh registration */
static const iocshArg ethercatmcCreateControllerArg0 = {"Port name",
                                                        iocshArgString};
static const iocshArg ethercatmcCreateControllerArg1 = {
    "EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg ethercatmcCreateControllerArg2 = {"Number of axes",
                                                        iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg3 = {
    "Moving poll period (ms)", iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg4 = {"Idle poll period (ms)",
                                                        iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg5 = {"options",
                                                        iocshArgString};
static const iocshArg *const ethercatmcCreateControllerArgs[] = {
    &ethercatmcCreateControllerArg0, &ethercatmcCreateControllerArg1,
    &ethercatmcCreateControllerArg2, &ethercatmcCreateControllerArg3,
    &ethercatmcCreateControllerArg4, &ethercatmcCreateControllerArg5};
static const iocshFuncDef ethercatmcCreateControllerDef = {
    strethercatmcCreateController, 6, ethercatmcCreateControllerArgs};
static void ethercatmcCreateContollerCallFunc(const iocshArgBuf *args) {
  ethercatmcCreateController(args[0].sval, args[1].sval, args[2].ival,
                             args[3].ival, args[4].ival, args[5].sval);
}

/* iocsh commands */
static const iocshArg ethercatmcCreateIndexerAxisArg0 = {"Controller port name",
                                                         iocshArgString};
static const iocshArg ethercatmcCreateIndexerAxisArg1 = {"Axis number",
                                                         iocshArgInt};
static const iocshArg ethercatmcCreateIndexerAxisArg2 = {"axisFlags",
                                                         iocshArgInt};
static const iocshArg ethercatmcCreateIndexerAxisArg3 = {"axisOptionsStr",
                                                         iocshArgString};
static const iocshArg *const ethercatmcCreateIndexerAxisArgs[] = {
    &ethercatmcCreateIndexerAxisArg0, &ethercatmcCreateIndexerAxisArg1,
    &ethercatmcCreateIndexerAxisArg2, &ethercatmcCreateIndexerAxisArg3};

static const iocshFuncDef ethercatmcCreateIndexerAxisDef = {
    strethercatmcCreateIndexerAxisDef, 4, ethercatmcCreateIndexerAxisArgs};

static const iocshArg ethercatmcCreateAsynParamArg0 = {"Controller port name",
                                                       iocshArgString};
static const iocshArg ethercatmcCreateAsynParamArg1 = {"Param Name",
                                                       iocshArgString};
static const iocshArg ethercatmcCreateAsynParamArg2 = {
    "Param Type: Float64 Int32", iocshArgString};
static const iocshArg *const ethercatmcCreateAsynParamArgs[] = {
    &ethercatmcCreateAsynParamArg0, &ethercatmcCreateAsynParamArg1,
    &ethercatmcCreateAsynParamArg2};

static const iocshArg ethercatmcStartPollerArg0 = {"Controller port name",
                                                   iocshArgString};
static const iocshArg ethercatmcStartPollerArg1 = {"Moving poll period (ms)",
                                                   iocshArgInt};
static const iocshArg ethercatmcStartPollerArg2 = {"Idle poll period (ms)",
                                                   iocshArgInt};

static const iocshArg *const ethercatmcStartPollerArgs[] = {
    &ethercatmcStartPollerArg0, &ethercatmcStartPollerArg1,
    &ethercatmcStartPollerArg2};

static const iocshFuncDef ethercatmcCreateAsynParamDef = {
    strethercatmcCreateAsynParamDef, 3, ethercatmcCreateAsynParamArgs};

static const iocshFuncDef ethercatmcStartPollerDef = {
    strethercatmcStartPollerDef, 3, ethercatmcStartPollerArgs};

static void ethercatmcCreateIndexerAxisCallFunc(const iocshArgBuf *args) {
  ethercatmcCreateIndexerAxis(args[0].sval, args[1].ival, args[2].ival,
                              args[3].sval);
}

static void ethercatmcCreateAsynParamCallFunc(const iocshArgBuf *args) {
  ethercatmcCreateAsynParam(args[0].sval, args[1].sval, args[2].sval);
}

static void ethercatmcStartPollerCallFunc(const iocshArgBuf *args) {
  ethercatmcStartPoller_C(args[0].sval, args[1].ival, args[2].ival);
}

static void ethercatmcControllerRegister(void) {
  iocshRegister(&ethercatmcCreateControllerDef,
                ethercatmcCreateContollerCallFunc);
  iocshRegister(&ethercatmcCreateIndexerAxisDef,
                ethercatmcCreateIndexerAxisCallFunc);
  iocshRegister(&ethercatmcCreateAsynParamDef,
                ethercatmcCreateAsynParamCallFunc);
  iocshRegister(&ethercatmcStartPollerDef, ethercatmcStartPollerCallFunc);
}

extern "C" {
epicsExportRegistrar(ethercatmcControllerRegister);
}

void ethercatmcController::setAlarmStatusSeverityAllReadbacks(
    asynStatus status) {
  int *pFunction = (int *)&defAsynPara;
  /*
    Set the status of all (asyn) functions that are predefined
    Note that functions may be defined here, but not used.
    So check for function != 0 before trying to set the status
  */

  for (size_t i = 0; i < sizeof(defAsynPara) / sizeof(int); i++, pFunction++) {
    int function = *pFunction;
    if (function) {
      setAlarmStatusSeverityAllAxes(function, status);
    }
  }

#ifdef motorMessageTextString
  setAlarmStatusSeverityAllAxes(motorMessageText_, status);
#endif
}

void ethercatmcController::setAlarmStatusSeverityAllAxes(int function,
                                                         asynStatus status) {
  for (int axisNo = 0; axisNo < numAxes_; axisNo++) {
    setAlarmStatusSeverityWrapper(axisNo, function, status);
  }
}

/* Alarm definition from EPICS Base */
#include <alarm.h>
/* Code inspired by .../asyn/devEpics/asynEpicsUtils.c */

void ethercatmcController::setAlarmStatusSeverityUpdate(int axisNo,
                                                        int function,
                                                        int newStat,
                                                        int newSevr) {
  const static char *const functionName = "AlarmStatSevr";
  /* alarm.h from EPICS base define these enums:
     epicsAlarmCondition epicsAlarmSeverity
  but we use "int" here */
  const char *paramName = NULL;
  if (getParamName(axisNo, function, &paramName)) paramName = "";

  int oldStat = -1;
  int oldSevr = -1;
  getParamAlarmStatus(axisNo, function, &oldStat);
  getParamAlarmSeverity(axisNo, function, &oldSevr);
  if (newStat != oldStat) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%s%s(%d) %s(%d) STAT=%s(%d)->%s(%d)\n", modNamEMC, functionName,
              axisNo, paramName, function, epicsAlarmConditionStrings[oldStat],
              oldStat, epicsAlarmConditionStrings[newStat], newStat);
    setParamAlarmStatus(axisNo, function, newStat);
  }
  if (newSevr != oldSevr) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%s%s(%d) %s(%d) SEVR=%s(%d)->%s(%d)\n", modNamEMC, functionName,
              axisNo, paramName, function, epicsAlarmSeverityStrings[oldSevr],
              oldSevr, epicsAlarmSeverityStrings[newSevr], newSevr);
    setParamAlarmSeverity(axisNo, function, newSevr);
  }
}

void ethercatmcController::setAlarmStatusSeverityWrapper(int axisNo,
                                                         int function,
                                                         asynStatus status) {
  /* alarm.h from EPICS base define these enums:
     epicsAlarmCondition epicsAlarmSeverity
  but we use "int" here */
  const char *paramName = NULL;
  if (getParamName(axisNo, function, &paramName)) paramName = "";

  int newSevr = INVALID_ALARM;
  int newStat = STATE_ALARM; /* Assume the worst */
  switch (status) {
    case asynSuccess:
      newSevr = NO_ALARM;
      newStat = NO_ALARM;
      break;
    case asynTimeout:
    case asynOverflow:
    case asynError:
    case asynDisabled:
    default:
      newSevr = INVALID_ALARM;
      newStat = STATE_ALARM; /* Assume the worst */
      break;
    case asynDisconnected:
      newSevr = INVALID_ALARM;
      newStat = COMM_ALARM;
      break;
  }
  setAlarmStatusSeverityUpdate(axisNo, function, newStat, newSevr);
}

void ethercatmcController::setAlarmStatusSeverityFromStatusBits(
    int axisNo, int function, epicsUInt32 statusReasonAux) {
  idxStatusCodeType idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
  unsigned idxReasonBits = (statusReasonAux >> 24) & 0x0F;

  int newSevr = INVALID_ALARM;
  int newStat = STATE_ALARM; /* Assume the worst */

  switch (idxStatusCode) {
    case idxStatusCodeRESET:
      newSevr = MINOR_ALARM;
      newStat = UDF_ALARM;
      break;
    case idxStatusCodeIDLE:
      /* Some MCU sends high/low even in IDLE */
      switch (idxReasonBits) {
        case 0x8:
          newSevr = MINOR_ALARM;
          newStat = HIGH_ALARM;
          break;
        case 0x4:
          newSevr = MINOR_ALARM;
          newStat = LOW_ALARM;
          break;
        case 0x2:
          newSevr = MINOR_ALARM;
          newStat = TIMEOUT_ALARM;
          break;
        case 0x1:
          newSevr = MINOR_ALARM;
          newStat = STATE_ALARM;
          break;
        case 0x0:
          newSevr = NO_ALARM;
          newStat = NO_ALARM;
          break;
        default:
          /* illegal/reserved combination */
          newSevr = MAJOR_ALARM;
          newStat = STATE_ALARM;
          break;
      }
      break;
    case idxStatusCodePOWEROFF:
    case idxStatusCodeWARN:
      newSevr = MINOR_ALARM;
      switch (idxReasonBits) {
        case 0x8:
          newStat = HIGH_ALARM;
          break;
        case 0x4:
          newStat = LOW_ALARM;
          break;
        case 0x2:
          newStat = TIMEOUT_ALARM;
          break;
        case 0x1:
          newStat = STATE_ALARM;
          break;
        case 0x0:
          newStat = STATE_ALARM;
          break;
        default:
          /* illegal/reserved combination */
          newSevr = MAJOR_ALARM;
          newStat = STATE_ALARM;
          break;
      }
      break;
    case idxStatusCodeSTART:
    case idxStatusCodeBUSY:
    case idxStatusCodeSTOP:
      /* temporary state, no action taken */
      return;
    case idxStatusCodeERROR:
    default:
      newSevr = INVALID_ALARM;
      newStat = STATE_ALARM; /* Assume the worst */
  }
  setAlarmStatusSeverityUpdate(axisNo, function, newStat, newSevr);
}
