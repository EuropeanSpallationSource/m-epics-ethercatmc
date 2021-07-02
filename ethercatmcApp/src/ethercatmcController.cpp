/*
FILENAME... ethercatmcController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsExit.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include "ethercatmcController.h"
#include "ethercatmcAxis.h"
#include "ethercatmcIndexerAxis.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

const char *modNamEMC = "ethercatmc:: ";

const static char *const strethercatmcCreateController = "ethercatmcCreateController";
const static char *const strethercatmcConfigController = "ethercatmcConfigController";
const static char *const strethercatmcConfigOrDie      = "ethercatmcConfigOrDie";
const static char *const strethercatmcReadController   = "ethercatmcReadController";
const static char *const strethercatmcCreateAxisDef    = "ethercatmcCreateAxis";
const static char *const strethercatmcCreateIndexerAxisDef = "ethercatmcCreateIndexerAxis";
const static char *const strethercatmcCreateAsynParamDef = "ethercatmcCreateAsynParam";
const static char *const strethercatmcStartPollerDef   = "ethercatmcStartPoller";
const static char *const strCtrlReset = ".ctrl.ErrRst";

const static char *const modulName = "ethercatmcAxis::";

const static unsigned reportedFeatureBits =
  FEATURE_BITS_SIM | FEATURE_BITS_ECMC |
  FEATURE_BITS_V1 | FEATURE_BITS_V2 | FEATURE_BITS_ADS;

extern "C"
double ethercatmcgetNowTimeSecs(void)
{
    epicsTimeStamp nowTime;
    epicsTimeGetCurrent(&nowTime);
    return nowTime.secPastEpoch + (nowTime.nsec * 0.000000001);
}

extern "C" const char *errStringFromErrId(int nErrorId)
{
  switch(nErrorId) {
  case 0x4221:
    return "Velo illegal";
  case 0x4223:
    return "Axis pos en";
  case 0x4450:
  case 0x4451:
    return "Follow err";
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
  case 0x4B09:
    return "Axis not rdy";
  case 0x4B0A:
    return "Homing faild";
  default:
    return "";
  }
}
extern "C" const char *stringFromAsynParamType(asynParamType paramType)
{
  switch (paramType) {
  case asynParamNotDefined: return "asynParamNotDefined";
  case asynParamInt32: return "asynParamInt32";
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
  case asynParamInt64: return "asynParamInt64";
#endif
  case asynParamUInt32Digital: return "asynParamUInt32Digital";
  case asynParamFloat64: return "asynParamFloat64";
  case asynParamOctet: return "asynParamOctet";
  case asynParamInt8Array: return "asynParamInt8Array";
  case asynParamInt16Array: return "asynParamInt16Array";
  case asynParamInt32Array: return "asynParamInt32Array";
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
  case asynParamInt64Array: return "asynParamInt64Array";
#endif
  case asynParamFloat32Array: return "asynParamFloat32Array";
  case asynParamFloat64Array: return "asynParamFloat64Array";
  case asynParamGenericPointer: return "asynParamGenericPointer";
  default: return "asynParamXXXXX";
  }
}

extern "C" const char *ethercatmcstrStatus(asynStatus status)
{
  switch ((int)status) {
  case asynSuccess:             return "asynSuccess";
  case asynTimeout:             return "asynTimeout";
  case asynOverflow:            return "asynOverflow";
  case asynError:               return "asynError";
  case asynDisconnected:        return "asynDisconnected";
  case asynDisabled:            return "asynDisabled";
  case asynParamAlreadyExists:  return "asynParamAlreadyExists";
  case asynParamNotFound:       return "asynParamNotFound";
  case asynParamWrongType:      return "asynParamWrongType";
  case asynParamBadIndex:       return "asynParamBadIndex";
  case asynParamUndefined:      return "asynParamUndefined";
  default: return "??";
  }
}

/** Creates a new ethercatmcController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the ethercatmc controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */

#if ETHERCATMC_ASYN_VERSION_INT < VERSION_INT_4_32
#error Need_asyn_later_than_4_32
#endif

ethercatmcController::ethercatmcController(const char *portName,
                                           const char *MotorPortName, int numAxes,
                                           double movingPollPeriod,
                                           double idlePollPeriod,
                                           const char *optionStr)
  :  asynMotorController(portName, numAxes,
                         0, // Olbsolete: Fixed number of additional asyn parameters
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
                         asynInt64Mask | asynUInt32DigitalMask | asynParamMetaMask, // additional interface
                         asynInt64Mask | asynUInt32DigitalMask | asynParamMetaMask, // additional callback interface
#else
                         asynParamMetaMask | asynUInt32DigitalMask,
                         asynParamMetaMask | asynUInt32DigitalMask,
#endif
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;

  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  ctrlLocal.oldStatus = asynError; //asynDisconnected;
  ctrlLocal.cntADSstatus = 0;
  features_ = 0;
#ifndef motorMessageTextString
  createParam("MOTOR_MESSAGE_TEXT",          asynParamOctet,       &ethercatmcMCUErrMsg_);
#else
  createParam(ethercatmcMCUErrMsgString,     asynParamOctet,       &ethercatmcMCUErrMsg_);
#endif
  createParam(ethercatmcDbgStrToMcuString,   asynParamOctet,       &ethercatmcDbgStrToMcu_);
  createParam(ethercatmcDbgStrToLogString,   asynParamOctet,       &ethercatmcDbgStrToLog_);
  createParam(ethercatmcDbgStrToNCString,    asynParamOctet,       &ethercatmcDbgStrToNC_);

  /* Per axis */
  createParam(ethercatmcErrString,           asynParamInt32,       &ethercatmcErr_);
  createParam(ethercatmcErrIdString,         asynParamInt32,       &ethercatmcErrId_);

  createParam(ethercatmcEnc_ActString,       asynParamFloat64,     &ethercatmcEncAct_);
  createParam(ethercatmcEnc_ActUTCString,    asynParamFloat64,     &ethercatmcEncActUTC_);
  createParam(ethercatmcHomProcString,       asynParamInt32,       &ethercatmcHomProc_);
  createParam(ethercatmcHomPosString,        asynParamFloat64,     &ethercatmcHomPos_);
  createParam(ethercatmcStatusCodeString,    asynParamInt32,       &ethercatmcStatusCode_);
  createParam(ethercatmcStatusBitsString,    asynParamUInt32Digital, &ethercatmcStatusBits_);
  createParam(ethercatmcFoffVisString,       asynParamInt32,       &ethercatmcFoffVis_);
  createParam(ethercatmcHomeVisString,       asynParamInt32,       &ethercatmcHomeVis_);
  createParam(ethercatmcHomProc_RBString,    asynParamInt32,       &ethercatmcHomProc_RB_);
  createParam(ethercatmcHomPos_RBString,     asynParamFloat64,     &ethercatmcHomPos_RB_);
  createParam(ethercatmcVelToHomString,      asynParamFloat64,     &ethercatmcVelToHom_);
  createParam(ethercatmcVelFrmHomString,     asynParamFloat64,     &ethercatmcVelFrmHom_);
  createParam(ethercatmcAccHomString,        asynParamFloat64,     &ethercatmcAccHom_);
  createParam(ethercatmcErrRstString,        asynParamInt32,       &ethercatmcErrRst_);
  createParam(ethercatmcVelActString,        asynParamFloat64,     &ethercatmcVelAct_);
  createParam(ethercatmcVel_RBString,        asynParamFloat64,     &ethercatmcVel_RB_);
  createParam(ethercatmcAcc_RBString,        asynParamFloat64,     &ethercatmcAcc_RB_);
  createParam(ethercatmcRBV_UTCString,       asynParamFloat64,     &ethercatmcRBV_UTC_);
  createParam(ethercatmcNamAux0_String,      asynParamOctet,       &ethercatmcNamAux0_);
  createParam(ethercatmcNamAux1_String,      asynParamOctet,       &ethercatmcNamAux1_);
  createParam(ethercatmcNamAux2_String,      asynParamOctet,       &ethercatmcNamAux2_);
  createParam(ethercatmcNamAux3_String,      asynParamOctet,       &ethercatmcNamAux3_);
  createParam(ethercatmcNamAux4_String,      asynParamOctet,       &ethercatmcNamAux4_);
  createParam(ethercatmcNamAux5_String,      asynParamOctet,       &ethercatmcNamAux5_);
  createParam(ethercatmcNamAux6_String,      asynParamOctet,       &ethercatmcNamAux6_);
  createParam(ethercatmcNamAux7_String,      asynParamOctet,       &ethercatmcNamAux7_);
  createParam(ethercatmcNamAux8_String,      asynParamOctet,       &ethercatmcNamAux8_);
  createParam(ethercatmcNamAux9_String,      asynParamOctet,       &ethercatmcNamAux9_);
  createParam(ethercatmcNamAux10_String,     asynParamOctet,       &ethercatmcNamAux10_);
  createParam(ethercatmcNamAux11_String,     asynParamOctet,       &ethercatmcNamAux11_);
  createParam(ethercatmcNamAux12_String,     asynParamOctet,       &ethercatmcNamAux12_);
  createParam(ethercatmcNamAux13_String,     asynParamOctet,       &ethercatmcNamAux13_);
  createParam(ethercatmcNamAux14_String,     asynParamOctet,       &ethercatmcNamAux14_);
  createParam(ethercatmcNamAux15_String,     asynParamOctet,       &ethercatmcNamAux15_);
  createParam(ethercatmcNamAux16_String,     asynParamOctet,       &ethercatmcNamAux16_);
  createParam(ethercatmcNamAux17_String,     asynParamOctet,       &ethercatmcNamAux17_);
  createParam(ethercatmcNamAux18_String,     asynParamOctet,       &ethercatmcNamAux18_);
  createParam(ethercatmcNamAux19_String,     asynParamOctet,       &ethercatmcNamAux19_);
  createParam(ethercatmcNamAux20_String,     asynParamOctet,       &ethercatmcNamAux20_);
  createParam(ethercatmcNamAux21_String,     asynParamOctet,       &ethercatmcNamAux21_);
  createParam(ethercatmcNamAux22_String,     asynParamOctet,       &ethercatmcNamAux22_);
  createParam(ethercatmcNamAux23_String,     asynParamOctet,       &ethercatmcNamAux23_);
  createParam(ethercatmcNamBit24_String,     asynParamOctet,       &ethercatmcNamBit24_);
  createParam(ethercatmcNamBit25_String,     asynParamOctet,       &ethercatmcNamBit25_);
  createParam(ethercatmcCfgAxisID_RBString,  asynParamInt32,       &ethercatmcCfgAxisID_RB_);
  createParam(ethercatmcCfgVELO_RBString,    asynParamFloat64,     &ethercatmcCfgVELO_RB_);
  createParam(ethercatmcCfgVMAX_RBString,    asynParamFloat64,     &ethercatmcCfgVMAX_RB_);
  createParam(ethercatmcCfgJVEL_RBString,    asynParamFloat64,     &ethercatmcCfgJVEL_RB_);
  createParam(ethercatmcCfgHVEL_RBString,    asynParamFloat64,     &ethercatmcCfgHVEL_RB_);
  createParam(ethercatmcCfgACCS_RBString,    asynParamFloat64,     &ethercatmcCfgACCS_RB_);
  createParam(ethercatmcCfgDHLMRBString,    asynParamFloat64,     &ethercatmcCfgDHLM_RB_);
  createParam(ethercatmcCfgDLLMRBString,    asynParamFloat64,     &ethercatmcCfgDLLM_RB_);
  createParam(ethercatmcCfgDHLM_EnRBString, asynParamInt32,       &ethercatmcCfgDHLM_En_RB_);
  createParam(ethercatmcCfgDLLM_EnRBString, asynParamInt32,       &ethercatmcCfgDLLM_En_RB_);
  createParam(ethercatmcCfgDHLMString,       asynParamFloat64,     &ethercatmcCfgDHLM_);
  createParam(ethercatmcCfgDLLMString,       asynParamFloat64,     &ethercatmcCfgDLLM_);
  createParam(ethercatmcCfgDHLM_EnString,    asynParamInt32,       &ethercatmcCfgDHLM_En_);
  createParam(ethercatmcCfgDLLM_EnString,    asynParamInt32,       &ethercatmcCfgDLLM_En_);

  createParam(ethercatmcCfgSREV_RBString,    asynParamFloat64,     &ethercatmcCfgSREV_RB_);
  createParam(ethercatmcCfgUREV_RBString,    asynParamFloat64,     &ethercatmcCfgUREV_RB_);
  createParam(ethercatmcCfgPMIN_RBString,    asynParamFloat64,     &ethercatmcCfgPMIN_RB_);
  createParam(ethercatmcCfgPMAX_RBString,    asynParamFloat64,     &ethercatmcCfgPMAX_RB_);
  createParam(ethercatmcCfgSPDB_RBString,    asynParamFloat64,     &ethercatmcCfgSPDB_RB_);
  createParam(ethercatmcCfgRDBD_RBString,    asynParamFloat64,     &ethercatmcCfgRDBD_RB_);
  createParam(ethercatmcCfgRDBD_Tim_RBString,asynParamFloat64,     &ethercatmcCfgRDBD_Tim_RB_);
  createParam(ethercatmcCfgRDBD_En_RBString, asynParamInt32,       &ethercatmcCfgRDBD_En_RB_);
  createParam(ethercatmcCfgPOSLAG_RBString,  asynParamFloat64,     &ethercatmcCfgPOSLAG_RB_);
  createParam(ethercatmcCfgPOSLAG_Tim_RBString,asynParamFloat64,   &ethercatmcCfgPOSLAG_Tim_RB_);
  createParam(ethercatmcCfgPOSLAG_En_RBString, asynParamInt32,     &ethercatmcCfgPOSLAG_En_RB_);

  createParam(ethercatmcCfgDESC_RBString,    asynParamOctet,       &ethercatmcCfgDESC_RB_);
  createParam(ethercatmcCfgEGU_RBString,     asynParamOctet,       &ethercatmcCfgEGU_RB_);

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);
#endif

  /* Connect to ethercatmc controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s cannot connect to motor controller\n", modulName);
  }
  printf("%s:%d %s optionStr=\"%s\"\n",
         __FILE__, __LINE__,
         modulName, optionStr ? optionStr : "NULL");
  if (optionStr && optionStr[0]) {
    const char * const adsPort_str        = "adsPort=";
    const char * const amsNetIdRemote_str = "amsNetIdRemote=";
    const char * const amsNetIdLocal_str  = "amsNetIdLocal=";
    char *pOptions = strdup(optionStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;
    int hasRemoteAmsNetId = 0;
    int hasLocalAmsNetId = 0;
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
        int nvals;
        memset(&ams_netid_port, 0, sizeof(ams_netid_port));
        pThisOption += strlen(amsNetIdRemote_str);
        nvals = sscanf(pThisOption, "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &ams_netid_port.netID[0],
                       &ams_netid_port.netID[1],
                       &ams_netid_port.netID[2],
                       &ams_netid_port.netID[3],
                       &ams_netid_port.netID[4],
                       &ams_netid_port.netID[5]);
        printf("%s:%d %s amsNetIdRemote=%u.%u.%u.%u.%u.%u:%u\n",
               __FILE__, __LINE__,
               modulName,
               ams_netid_port.netID[0], ams_netid_port.netID[1],
               ams_netid_port.netID[2], ams_netid_port.netID[3],
               ams_netid_port.netID[4], ams_netid_port.netID[5],
               ctrlLocal.adsport);
        if (nvals == 6) {
          hasRemoteAmsNetId = 1;
          ams_netid_port.port_low  = (uint8_t)ctrlLocal.adsport;
          ams_netid_port.port_high = (uint8_t)(ctrlLocal.adsport >> 8);
          memcpy(&ctrlLocal.remote, &ams_netid_port, sizeof(ctrlLocal.remote));
        }
      } else if (!strncmp(pThisOption, amsNetIdLocal_str,
                          strlen(amsNetIdLocal_str))) {
        AmsNetidAndPortType ams_netid_port;
        int nvals;
        memset(&ams_netid_port, 0, sizeof(ams_netid_port));
        pThisOption += strlen(amsNetIdLocal_str);
        nvals = sscanf(pThisOption, "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &ams_netid_port.netID[0],
                       &ams_netid_port.netID[1],
                       &ams_netid_port.netID[2],
                       &ams_netid_port.netID[3],
                       &ams_netid_port.netID[4],
                       &ams_netid_port.netID[5]);
        printf("%s:%d %s amsNetIdLocal=%u.%u.%u.%u.%u.%u:%u\n",
               __FILE__, __LINE__,
               modulName,
               ams_netid_port.netID[0], ams_netid_port.netID[1],
               ams_netid_port.netID[2], ams_netid_port.netID[3],
               ams_netid_port.netID[4], ams_netid_port.netID[5],
               ctrlLocal.adsport);
        if (nvals == 6) {
          hasLocalAmsNetId = 1;
          ams_netid_port.port_low  = (uint8_t)ctrlLocal.adsport;
          ams_netid_port.port_high = (uint8_t)(ctrlLocal.adsport >> 8);
          memcpy(&ctrlLocal.local, &ams_netid_port, sizeof(ctrlLocal.local));
        }
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
    if (hasRemoteAmsNetId && hasLocalAmsNetId) {
      ctrlLocal.useADSbinary = 1;
    }
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_INFO,
            "%s optionStr=\"%s\"\n",
            modulName, optionStr ? optionStr : "NULL");

  if (!(ctrlLocal.useADSbinary)) {
    /* Set the EOS */
    asynUser *pasynUser = pasynUserController_;
    const char *pMyEos = ";\n";
    status = pasynOctetSyncIO->setInputEos(pasynUser, pMyEos, strlen(pMyEos));
    if (status) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sstatus=%s (%d)\n", modNamEMC,
                ethercatmcstrStatus(status), (int)status);
    }
    status = pasynOctetSyncIO->setOutputEos(pasynUser, pMyEos, strlen(pMyEos));
    if (status) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sstatus=%s (%d)\n", modNamEMC,
                ethercatmcstrStatus(status), (int)status);
    }
  } else {
    if (movingPollPeriod && idlePollPeriod) {
      /*  Find additional devices/asynParams */
      poll();
    }
  }
  if (movingPollPeriod && idlePollPeriod) {
    startPoller(movingPollPeriod, idlePollPeriod, 2);
  }
}


/** Creates a new ethercatmcController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the ethercatmc controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int ethercatmcCreateController(const char *portName,
                                          const char *MotorPortName,
                                          int numAxes,
                                          int movingPollPeriod,
                                          int idlePollPeriod,
                                          const char *optionStr)
{
  new ethercatmcController(portName, MotorPortName, 1+numAxes,
                           movingPollPeriod/1000., idlePollPeriod/1000.,
                           optionStr);
  return(asynSuccess);
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
extern "C" int ethercatmcConfigController(int needOkOrDie, const char *portName,
                                          const char *configStr)
{
  ethercatmcController *pC;

  if (!portName || !configStr)  {
    printf("%sNULL parameter\n", strethercatmcConfigController);
    return asynError;
  }
  pC = (ethercatmcController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           __FILE__, __FUNCTION__, portName);
    return asynError;
  }
  return pC->configController(needOkOrDie, configStr);
}

asynStatus ethercatmcController::configController(int needOkOrDie, const char *value)
{
  char inString[MAX_CONTROLLER_STRING_SIZE];
  size_t configStrLen = strlen(value);
  asynStatus status = asynError;

  if (!strcmp(value, strCtrlReset)) {
    ctrlLocal.hasConfigError = 0;
    setMCUErrMsg("OK");
    return asynSuccess;
  }
  if (ctrlLocal.hasConfigError) {
    printf("port %s has errors. To reset use\n %s %s %s \n",
           portName, strethercatmcConfigController, portName, strCtrlReset);
    return asynError;
  }

  status = writeReadOnErrorDisconnect_C(pasynUserController_,
                                        value, configStrLen,
                                        inString, sizeof(inString));
  inString[sizeof(inString) -1] = '\0';
  if (status) {
    ;
  } else if (needOkOrDie) {
    status = checkACK(value, configStrLen, inString);
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s out=%s in=\"%s\" return=%s (%d)\n",
                modulName, value, inString,
                ethercatmcstrStatus(status), (int)status);
      if (needOkOrDie < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s Aborting IOC\n",
                  modulName);
        epicsExit(EXIT_FAILURE);
      }
      ctrlLocal.hasConfigError = 1;
      (void)setMCUErrMsg(inString);
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_INFO,
                "%s out=%s in=\"%s\"\n",
                modulName, value, inString);
    }
  } /* needOkOrDie */

  printf("%s\n", inString);
  return status;
}


asynStatus
ethercatmcController::ethercatmcCreateParam(const char *paramName,
                                            asynParamType myEPICSParamType,
                                            int *pFunction)
{
  asynStatus status;
  status = createParam(paramName,
                       myEPICSParamType,
                       pFunction);

  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s paramName=%s paramType=%s function=%d status=%s (%d)\n",
            modNamEMC, paramName,
            stringFromAsynParamType(myEPICSParamType),
            *pFunction,
            ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus ethercatmcController::ethercatmcStartPoller(double movingPollPeriod,
                                                       double idlePollPeriod)
{
  return startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 2);
}

extern "C" int ethercatmcCreateAsynParam(const char *ethercatmcName,
                                         const char *paramName,
                                         const char *paramType)
{
  ethercatmcController *pC;
  int newFunction = 0;
  asynParamType myEPICSParamType;

  if (!ethercatmcName || !paramName || !paramType) {
    printf("ethercatmcCreateAsynParam MCU1 paramName [Float64|Int32|Int64]\n");
    return asynError;
  }
  pC = (ethercatmcController*) findAsynPortDriver(ethercatmcName);
  if (!pC) {
    printf("Error port %s not found\n", ethercatmcName);
    return asynError;
  }
  if (!strcmp(paramType, "Float64")) {
    myEPICSParamType = asynParamFloat64;
  } else if (!strcmp(paramType, "Int32")) {
    myEPICSParamType = asynParamInt32;
  } else if (!strcmp(paramType, "Int64")) {
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
    myEPICSParamType = asynParamInt64;
#else
    myEPICSParamType = asynParamFloat64;
#endif
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
                                       int idlePollPeriod)
{
  ethercatmcController *pC;

  if (!ethercatmcName) {
    printf("ethercatmcStartPoller MCU1\n");
    return asynError;
  }
  pC = (ethercatmcController*) findAsynPortDriver(ethercatmcName);
  if (!pC) {
    printf("ethercatmcStartPoller: Error port %s not found\n", ethercatmcName);
    return asynError;
  }
  return pC->ethercatmcStartPoller((double)movingPollPeriod,
                                   (double)idlePollPeriod);
}

extern "C" asynStatus disconnect_C(asynUser *pasynUser)
{
  asynStatus status = asynError;
  asynInterface *pasynInterface = NULL;
  asynCommon     *pasynCommon = NULL;
  pasynInterface = pasynManager->findInterface(pasynUser,
                                               asynCommonType,
                                               0 /* FALSE */);
  if (pasynInterface) {
    pasynCommon = (asynCommon *)pasynInterface->pinterface;
    status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUser);
    if (status != asynSuccess) {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s status=%s (%d)\n",
                modulName, ethercatmcstrStatus(status), (int)status);
    }
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s pasynInterface=%p pasynCommon=%p\n",
              modulName, pasynInterface, pasynCommon);
  }
  return status;
}

extern "C"
asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                        const char *outdata, size_t outlen,
                                        char *indata, size_t inlen)
{
  size_t nwrite;
  asynStatus status = asynError;
  int eomReason = 0;
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUser, outdata, outlen,
                                       indata, inlen,
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) ||
      (!nread && (eomReason & ASYN_EOM_END))) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s out=%s nread=%lu eomReason=%x (%s%s%s) status=%s (%d)\n",
              modulName, outdata,(unsigned long)nread,
              eomReason,
              eomReason & ASYN_EOM_CNT ? "CNT" : "",
              eomReason & ASYN_EOM_EOS ? "EOS" : "",
              eomReason & ASYN_EOM_END ? "END" : "",
              ethercatmcstrStatus(status), status);
    disconnect_C(pasynUser);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus ethercatmcController::writeReadOnErrorDisconnect(void)
{
  asynStatus status = asynError;
  size_t outlen = strlen(outString_);
  double timeBefore = ethercatmcgetNowTimeSecs();
  inString_[0] = '\0';
  status = writeReadOnErrorDisconnect_C(pasynUserController_, outString_, outlen,
                                        inString_, sizeof(inString_));
  if (!status) {
    if (strstr(inString_, "State timout") ||
        strstr(inString_, "To long time in one state")) {
      double timeDelta = ethercatmcgetNowTimeSecs() - timeBefore;

      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sout=%s in=%s timeDelta=%f\n",
                modNamEMC, outString_, inString_, timeDelta);
      /* Try again */
      timeBefore = ethercatmcgetNowTimeSecs();
      status = writeReadOnErrorDisconnect_C(pasynUserController_,
                                            outString_, outlen,
                                            inString_, sizeof(inString_));
      timeDelta = ethercatmcgetNowTimeSecs() - timeBefore;
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%sout=%s in=%s timeDelta=%f status=%s (%d)\n",
                modNamEMC, outString_, inString_, timeDelta,
                ethercatmcstrStatus(status), (int)status);
    }
  }
  handleStatusChange(status);
  if (status)
  {
    return asynError;
  }
  return status;
}

extern "C"
asynStatus checkACK(const char *outdata, size_t outlen,
                    const char *indata)
{
  size_t i;
  unsigned int numOK = 1;
  int res = 1;
  for( i = 0; i < outlen; i++) {
    if (outdata[i] == ';') numOK++;
  }
  switch(numOK) {
    case 1: res = strcmp(indata, "OK");  break;
    case 2: res = strcmp(indata, "OK;OK");  break;
    case 3: res = strcmp(indata, "OK:OK;OK");  break;
    case 4: res = strcmp(indata, "OK;OK;OK;OK");  break;
    default:
    ;
  }
  return res ? asynError : asynSuccess;
}

asynStatus ethercatmcController::writeReadControllerPrint(int traceMask)
{
  asynStatus status = writeReadOnErrorDisconnect();
  if (status && !ctrlLocal.oldStatus) traceMask |= ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER;
  asynPrint(pasynUserController_, traceMask,
            "%sout=%s in=%s status=%s (%d)\n",
            modNamEMC, outString_, inString_,
            ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus ethercatmcController::writeReadACK(int traceMask)
{
  asynStatus status = writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
    {
      const char *semicolon = &outString_[0];
      unsigned int numOK = 1;
      int res = 1;
      while (semicolon && semicolon[0]) {
        semicolon = strchr(semicolon, ';');
        if (semicolon) {
          numOK++;
          semicolon++;
        }
      }
      switch(numOK) {
        case 1: res = strcmp(inString_, "OK");  break;
        case 2: res = strcmp(inString_, "OK;OK");  break;
        case 3: res = strcmp(inString_, "OK:OK;OK");  break;
        case 4: res = strcmp(inString_, "OK;OK;OK;OK");  break;
        case 5: res = strcmp(inString_, "OK;OK;OK;OK;OK");  break;
        case 6: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK");  break;
        case 7: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK;OK");  break;
        case 8: res = strcmp(inString_, "OK;OK;OK;OK;OK;OK;OK;OK");  break;
        default:
          ;
      }
      if (res) {
        status = asynError;
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%sout=%s in=%s return=%s (%d)\n",
                  modNamEMC, outString_, inString_,
                  ethercatmcstrStatus(status), (int)status);
        return status;
      }
    }
    default:
      break;
  }
  asynPrint(pasynUserController_, traceMask,
            "%sout=%s in=%s status=%s (%d)\n",
            modNamEMC, outString_, inString_,
            ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus ethercatmcController::setMCUErrMsg(const char *value)
{
  asynStatus status = setStringParam(ethercatmcMCUErrMsg_, value);
  if (!status) status = callParamCallbacks();
  return status;
}

void ethercatmcController::udateMotorLimitsRO(int axisNo)
{
  double fValueHigh = 0.0, fValueLow = 0.0;
  int enabledHigh = 0, enabledLow = 0;

  /* When the integer parameter is undefined, 0 is returned,
     same as not enabled */
  getIntegerParam(axisNo, ethercatmcCfgDHLM_En_RB_, &enabledHigh);
  getIntegerParam(axisNo, ethercatmcCfgDLLM_En_RB_, &enabledLow);

  if (enabledHigh && enabledLow) {
    asynStatus status1, status2;
    status1 = getDoubleParam(axisNo, ethercatmcCfgDHLM_RB_, &fValueHigh);
    status2 = getDoubleParam(axisNo, ethercatmcCfgDLLM_RB_, &fValueLow);

    if (status1 || status2) {
      udateMotorLimitsRO(axisNo, 0, 0.0, 0.0);
      return;
    }
  }
  udateMotorLimitsRO(axisNo, enabledHigh && enabledLow, fValueHigh, fValueLow);
}

void ethercatmcController::udateMotorLimitsRO(int axisNo, int enabledHighAndLow,
                                              double fValueHigh, double fValueLow)
{
#ifdef motorHighLimitROString
  static const double fABSMIN = -3.0e+38;
  static const double fABSMAX =  3.0e+38;
  int valid = 1;
  if (fValueLow >= fValueHigh ||
      (fValueLow <= fABSMIN) || (fValueHigh >= fABSMAX)) {
    /* Any limit not active or out of range: set everything to 0 */
    valid = 0;
  }

  if (!enabledHighAndLow || !valid) {
    /* Any limit not active or out of range: set everything to 0 */
    fValueHigh = fValueLow  = 0.0;
  }
  asynMotorAxis *pAxis = getAxis(axisNo);
  if (pAxis) {
    double oldValueHigh, oldValueLow;
    getDoubleParam(axisNo, motorHighLimitRO_, &oldValueHigh);
    getDoubleParam(axisNo, motorLowLimitRO_,  &oldValueLow);
    if ((fValueHigh != oldValueHigh) || (fValueLow != oldValueLow)) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sudateMotorLimitsRO(%d) enabledHighAndLow=%d valid=%d fValueHigh=%g fValueLow=%g\n",
                modNamEMC, axisNo,
                enabledHighAndLow, valid, fValueHigh, fValueLow);
    }


    /* We need the overload function from asynMotorAxis to
       let the values ripple into the motorRecord */
    pAxis->setDoubleParam(motorHighLimitRO_, fValueHigh);
    pAxis->setDoubleParam(motorLowLimitRO_,  fValueLow);
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
                                                int lineNo)
{
  if (status != ctrlLocal.oldStatus) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s %s:%d %s oldStatus=%s(%d) newStatus=%s(%d)\n",
              modNamEMC, fileName, lineNo, "handleStatusChange",
              ethercatmcstrStatus(ctrlLocal.oldStatus), (int)ctrlLocal.oldStatus,
              ethercatmcstrStatus(status), (int)status);
    if (status) {
      /* Connected -> Disconnected */
      int axisNo;
      ctrlLocal.initialPollDone = 0;
      setAlarmStatusSeverityAllReadbacks(asynDisconnected);
      if (ctrlLocal.useADSbinary) {
        indexerDisconnected();
      }

      /* Keep bits that are specified via options,
         clear bits that are fetched from the controller */
      features_ &= ~reportedFeatureBits;
      setMCUErrMsg("MCU Disconnected");
      for (axisNo=0; axisNo<numAxes_; axisNo++) {
        asynMotorAxis *pAxis=getAxis(axisNo);
        if (!pAxis) continue;
        pAxis->setIntegerParam(motorStatusCommsError_, 1);
        setIntegerParam(axisNo, ethercatmcFoffVis_, 0);
        setIntegerParam(axisNo, ethercatmcHomeVis_, 0);
        pAxis->callParamCallbacks();
      }
    } else {
      /* Disconnected -> Connected */
      setMCUErrMsg("MCU Cconnected");
      ctrlLocal.cntADSstatus = 0;
    }
    ctrlLocal.oldStatus = status;
  }
}

asynStatus ethercatmcController::poll(void)
{
  asynStatus status = asynSuccess;

  asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
            "%spoll ctrlLocal.initialPollDone=%d features_=0x%x\n",
            modNamEMC, ctrlLocal.initialPollDone, features_);

  if (ctrlLocal.useADSbinary) {
    if (!ctrlLocal.initialPollDone) {
      status = indexerInitialPoll();
      if (!status) {
        handleStatusChange(status);
        ctrlLocal.initialPollDone = 1;
      } else {
        int i = 1;
        while (i < numAxes_) {
          setIntegerParam(i ,motorStatusCommsError_, 1);
          callParamCallbacks(i);
          i++;
        }
      }
    } else {
      status = indexerPoll();
      if (status) {
        handleStatusChange(status);
      }
      return status;
    }
  } else {
    if (!(features_ & reportedFeatureBits)) {
      int reportedFeatures;
      status = getFeatures(&reportedFeatures);
      if (!status) {
        features_ |= reportedFeatures;
        ctrlLocal.initialPollDone = 1;
      }
    }
  }
  return status;
}

asynStatus ethercatmcController::updateCfgValue(int axisNo_, int function,
                                                double newValue, const char *name)
{
  double oldValue;
  asynStatus status = getDoubleParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!(features_ & FEATURE_BITS_ECMC)) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%f\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%f new%s=%f\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
  return setDoubleParam(axisNo_, function, newValue);
}

asynStatus ethercatmcController::updateCfgValue(int axisNo_, int function,
                                                int newValue, const char *name)
{
  int oldValue;
  asynStatus status = getIntegerParam(axisNo_, function, &oldValue);
  if (status) {
    /* First time that we write the value after IOC restart
       ECMC configures everything from the iocshell, no need to
       do a print here */
    if (!((features_ & FEATURE_BITS_ECMC))) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%supdateCfgValue(%d) %s=%d\n",
                modNamEMC, axisNo_, name, newValue);
    }
  } else if (newValue != oldValue) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%supdateCfgValue(%d) old%s=%d new%s=%d\n",
              modNamEMC, axisNo_, name, oldValue, name, newValue);
  }
  setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
  return setIntegerParam(axisNo_, function, newValue);
}


asynStatus ethercatmcController::getFeatures(int *pRet)
{
  /* The features we know about */
  const char * const sim_str = "sim";
  const char * const stECMC_str = "ecmc";
  const char * const stV1_str = "stv1";
  const char * const stV2_str = "stv2";
  const char * const ads_str = "ads";
  static const unsigned adsports[] = {851, 852, 853};
  unsigned adsport_idx;
  *pRet = 0;
  int ret = 0;
  for (adsport_idx = 0;
       adsport_idx < sizeof(adsports)/sizeof(adsports[0]);
       adsport_idx++) {
    unsigned adsport = adsports[adsport_idx];

    asynStatus status = asynSuccess;
    snprintf(outString_, sizeof(outString_),
             "ADSPORT=%u/.THIS.sFeatures?",
             adsport);
    inString_[0] = 0;
    status = writeReadOnErrorDisconnect();
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%sout=%s in=%s status=%s (%d)\n",
              modNamEMC, outString_, inString_,
              ethercatmcstrStatus(status), (int)status);
    if (!status) {
      /* loop through the features */
      char *pAllFeatures = strdup(inString_);
      char *pThisFeature = pAllFeatures;
      char *pNextFeature = pAllFeatures;

      while (pNextFeature && pNextFeature[0]) {
        pNextFeature = strchr(pNextFeature, ';');
        if (pNextFeature) {
          *pNextFeature = '\0'; /* Terminate */
          pNextFeature++;       /* Jump to (possible) next */
        }
        if (!strcmp(pThisFeature, sim_str)) {
          ret |= FEATURE_BITS_SIM;
        } else if (!strcmp(pThisFeature, stECMC_str)) {
          ret |= FEATURE_BITS_ECMC;
        } else if (!strcmp(pThisFeature, stV1_str)) {
          ret |= FEATURE_BITS_V1;
        } else if (!strcmp(pThisFeature, stV2_str)) {
          ret |= FEATURE_BITS_V2;
        } else if (!strcmp(pThisFeature, ads_str)) {
          ret |= FEATURE_BITS_ADS;
        }
        pThisFeature = pNextFeature;
      }
      free(pAllFeatures);
      if (ret) {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sout=%s in=%s ret=0x%x\n",
                  modNamEMC, outString_, inString_,
                  ret);
        /* Found something useful on this adsport */
        *pRet = ret;
        return asynSuccess;
      }
    }
  }
  return asynError;
}

asynStatus ethercatmcController::readOctet(asynUser *pasynUser, char *value,
                                           size_t maxChars, size_t *nActual,
                                           int *eomReason)
{
  static const char *functionName = "readOctet";
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;

  if (!function || (function == ethercatmcDbgStrToNC_)) {
    /*
     * There are 2 ways to end up here:
     * - e.g. an bo Record using StreamDevice with a proto file
     *   doing like this:
     *   setDHLM_En {
     *      out "ADSPORT=501/.ADR.16#5001,16#B,2,2=%d;";
     *   }
     *   Then we will see function == 0 here
     *   Note: The StreamDevice must talk to the MOTOR_PORT,
     *   not the ASYN_PORT.
     * - A asyn stringout (or waveform of chars) connected to "StrToNC"
     *
     */
    int addr = 0;
    function = ethercatmcDbgStrToNC_;
    status = getStringParam(addr, function, (int)maxChars, value);
    setStringParam(addr, function, "NoResonse;");
    return status;
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s function=%d\n",
            modNamEMC, functionName, function);
  return asynPortDriver::readOctet(pasynUser, value,
                                   maxChars, nActual, eomReason);
}

/** Called when asyn clients call pasynOctetSyncIO->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write.
  * \param[in] nChars len (but we only support strings ?!).
  * \param[out] nActual. number of octets that had been written */
asynStatus ethercatmcController::writeOctet(asynUser *pasynUser,
                                            const char *value,
                                            size_t nChars, size_t *nActual)
{
  static const char *functionName = "writeOctet";
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  int function = pasynUser->reason;
  pAxis = getAxis(pasynUser);

  *nActual = nChars; /* All data went out */
  if (!function || (function == ethercatmcDbgStrToNC_)) {
    /* Streamdevice special handling */
    int      addr = 0;
    int      nvals;
    unsigned adsport;
    unsigned idxGrp = 0;
    unsigned idxOff;
    unsigned lenInPlc;
    unsigned typInPlc;
    double   fValInPlc;
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s function=%d value=%s\n",
              modNamEMC, functionName, function, value);

    nvals = sscanf(value, "ADSPORT=%u/.ADR.16#%x,16#%x,%u,%u=%lf;",
                   &adsport, &idxGrp, &idxOff,
                   &lenInPlc, &typInPlc, &fValInPlc);
    if (nvals != 6) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s function=%d value=%s Bad String\n",
                modNamEMC, functionName, function, value);
      return asynError;
    } else {
      unsigned requestedAxisID = idxGrp & 0xFF;
      int      axisAxisID = 0;
      asynMotorAxis *pAxis = getAxis(requestedAxisID);
      status = pAxis ? asynSuccess : asynError;
      if (!status) {
        getIntegerParam(requestedAxisID, ethercatmcCfgAxisID_RB_, &axisAxisID);
        if ((unsigned)axisAxisID != requestedAxisID) status = asynError;
      }
      if (status) {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s Bad idxGrp requestedAxisID=%u pAxis=%p axisAxisID=%d\n",
                  modNamEMC, functionName, requestedAxisID, pAxis, axisAxisID);
        return asynError;
      }
    }
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s adsport=%u idxGrp=0x%X idxOff=0x%X lenInPlc=%u typInPlc=%u fValInPlc=%f\n",
              modNamEMC, functionName,
              adsport, idxGrp, idxOff, lenInPlc, typInPlc, fValInPlc);

    if (ctrlLocal.useADSbinary) {
      if (adsport == 501 && typInPlc == 2 && lenInPlc == 2) {
        status = setSAFIntegerOnAxisViaADS(idxGrp, idxOff, (int)fValInPlc, lenInPlc);
        setStringParam(addr, ethercatmcDbgStrToNC_, status ? "Error;" : "OK;");
        return status;
      } else if (adsport == 501 && typInPlc == 5 && lenInPlc == 8) {
        status = setSAFDoubleOnAxisViaADS(idxGrp, idxOff, fValInPlc, lenInPlc);
        setStringParam(addr, ethercatmcDbgStrToNC_, status ? "Error;" : "OK;");
        return status;
      }
    }
    /* If we come here: Something went wrong */
    setStringParam(addr, ethercatmcDbgStrToNC_, value);
    return asynError;
  }

  if (!pAxis) return asynError;

  status = pAxis->setStringParam(function, value);
  if (status == asynSuccess) *nActual = strlen(value);
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void ethercatmcController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an ethercatmcAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
ethercatmcAxis* ethercatmcController::getAxis(asynUser *pasynUser)
{
  return static_cast<ethercatmcAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an ethercatmcAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
ethercatmcAxis* ethercatmcController::getAxis(int axisNo)
{
  return static_cast<ethercatmcAxis*>(asynMotorController::getAxis(axisNo));
}


/** Code for iocsh registration */
static const iocshArg ethercatmcCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ethercatmcCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg ethercatmcCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg ethercatmcCreateControllerArg5 = {"options", iocshArgString};
static const iocshArg *const ethercatmcCreateControllerArgs[] = {&ethercatmcCreateControllerArg0,
                                                            &ethercatmcCreateControllerArg1,
                                                            &ethercatmcCreateControllerArg2,
                                                            &ethercatmcCreateControllerArg3,
                                                            &ethercatmcCreateControllerArg4,
                                                            &ethercatmcCreateControllerArg5};
static const iocshFuncDef ethercatmcCreateControllerDef = {strethercatmcCreateController, 6,
                                                           ethercatmcCreateControllerArgs};
static void ethercatmcCreateContollerCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateController(args[0].sval, args[1].sval, args[2].ival,
                             args[3].ival, args[4].ival, args[5].sval);
}

/* ethercatmcConfigController */
static const iocshArg ethercatmcConfigControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg ethercatmcConfigControllerArg1 = {"ConfigString",         iocshArgString};
static const iocshArg * const ethercatmcConfigControllerArgs[] = {&ethercatmcConfigControllerArg0,
                                                                  &ethercatmcConfigControllerArg1};
static const iocshFuncDef ethercatmcConfigControllerDef = {strethercatmcConfigController, 2,
                                                           ethercatmcConfigControllerArgs};
static const iocshFuncDef ethercatmcConfigOrDieDef = {strethercatmcConfigOrDie, 2,
                                                      ethercatmcConfigControllerArgs};
static const iocshFuncDef ethercatmcReadControllerDef = {strethercatmcReadController, 2,
                                                         ethercatmcConfigControllerArgs};

static void ethercatmcConfigContollerCallFunc(const iocshArgBuf *args)
{
  int needOkOrDie = 1;
  ethercatmcConfigController(needOkOrDie, args[0].sval, args[1].sval);
}

static void ethercatmcConfigOrDieCallFunc(const iocshArgBuf *args)
{
  int needOkOrDie = -1;
  ethercatmcConfigController(needOkOrDie, args[0].sval, args[1].sval);
}

static void ethercatmcReadContollerCallFunc(const iocshArgBuf *args)
{
  int needOkOrDie = 0;
  ethercatmcConfigController(needOkOrDie, args[0].sval, args[1].sval);
}



/* ethercatmcCreateAxis */
static const iocshArg ethercatmcCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg ethercatmcCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg ethercatmcCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg ethercatmcCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const ethercatmcCreateAxisArgs[] = {&ethercatmcCreateAxisArg0,
                                                            &ethercatmcCreateAxisArg1,
                                                            &ethercatmcCreateAxisArg2,
                                                            &ethercatmcCreateAxisArg3};
static const
iocshArg * const ethercatmcCreateIndexerAxisArgs[] = {&ethercatmcCreateAxisArg0,
                                                      &ethercatmcCreateAxisArg1,
                                                      &ethercatmcCreateAxisArg2,
                                                      &ethercatmcCreateAxisArg3};

static const iocshFuncDef ethercatmcCreateAxisDef = {strethercatmcCreateAxisDef, 4,
                                                     ethercatmcCreateAxisArgs};

static const iocshFuncDef ethercatmcCreateIndexerAxisDef = {strethercatmcCreateIndexerAxisDef, 4,
                                                     ethercatmcCreateIndexerAxisArgs};

static const iocshArg ethercatmcCreateAsynParamArg0 = {"Controller port name", iocshArgString};
static const iocshArg ethercatmcCreateAsynParamArg1 = {"Param Name", iocshArgString};
static const iocshArg ethercatmcCreateAsynParamArg2 = {"Param Type: Float64 Int32", iocshArgString};
static const iocshArg * const ethercatmcCreateAsynParamArgs[] = {&ethercatmcCreateAsynParamArg0,
                                                                 &ethercatmcCreateAsynParamArg1,
                                                                 &ethercatmcCreateAsynParamArg2};

static const iocshArg ethercatmcStartPollerArg0 = {"Controller port name", iocshArgString};
static const iocshArg ethercatmcStartPollerArg1 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ethercatmcStartPollerArg2 = {"Idle poll period (ms)", iocshArgInt};

static const iocshArg * const ethercatmcStartPollerArgs[] = {&ethercatmcStartPollerArg0,
                                                             &ethercatmcStartPollerArg1,
                                                             &ethercatmcStartPollerArg2};

static const iocshFuncDef ethercatmcCreateAsynParamDef = {strethercatmcCreateAsynParamDef, 3,
                                                          ethercatmcCreateAsynParamArgs};
static const iocshFuncDef ethercatmcStartPollerDef = {strethercatmcStartPollerDef, 3,
                                                      ethercatmcStartPollerArgs};
static void ethercatmcCreateAxisCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ethercatmcCreateIndexerAxisCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateIndexerAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ethercatmcCreateAsynParamCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateAsynParam(args[0].sval, args[1].sval, args[2].sval);
}

static void ethercatmcStartPollerCallFunc(const iocshArgBuf *args)
{
  ethercatmcStartPoller_C(args[0].sval, args[1].ival, args[2].ival);
}

static void ethercatmcControllerRegister(void)
{
  iocshRegister(&ethercatmcCreateControllerDef, ethercatmcCreateContollerCallFunc);
  iocshRegister(&ethercatmcConfigOrDieDef,      ethercatmcConfigOrDieCallFunc);
  iocshRegister(&ethercatmcConfigControllerDef, ethercatmcConfigContollerCallFunc);
  iocshRegister(&ethercatmcReadControllerDef,   ethercatmcReadContollerCallFunc);
  iocshRegister(&ethercatmcCreateAxisDef,       ethercatmcCreateAxisCallFunc);
  iocshRegister(&ethercatmcCreateIndexerAxisDef,ethercatmcCreateIndexerAxisCallFunc);
  iocshRegister(&ethercatmcCreateAsynParamDef,  ethercatmcCreateAsynParamCallFunc);
  iocshRegister(&ethercatmcStartPollerDef,      ethercatmcStartPollerCallFunc);
}

extern "C" {
  epicsExportRegistrar(ethercatmcControllerRegister);
}

void ethercatmcController::setAlarmStatusSeverityAllReadbacks(asynStatus status)
{
  setAlarmStatusSeverityAllAxes(ethercatmcEncAct_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcEncActUTC_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcHomProc_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcHomPos_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcVel_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcAcc_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcRBV_UTC_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgDHLM_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgDLLM_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgDHLM_En_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgDLLM_En_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgSREV_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgUREV_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgPMIN_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgPMAX_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgSPDB_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgRDBD_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgRDBD_Tim_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgRDBD_En_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgPOSLAG_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgPOSLAG_Tim_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgPOSLAG_En_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgDESC_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgEGU_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgVELO_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgVMAX_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgJVEL_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgHVEL_RB_, status);
  setAlarmStatusSeverityAllAxes(ethercatmcCfgACCS_RB_, status);

}

void ethercatmcController::setAlarmStatusSeverityAllAxes(int function, asynStatus status)
{
  for (int axisNo=0; axisNo<numAxes_; axisNo++) {
    setAlarmStatusSeverityWrapper(axisNo, function,  status);
  }
}


/* Alarm definition from EPICS Base */
#include <alarm.h>
/* Code inspired by .../asyn/devEpics/asynEpicsUtils.c */
void ethercatmcController::setAlarmStatusSeverityWrapper(int axisNo,
                                                         int function,
                                                         asynStatus status)
{
  const static char *const functionName = "AlarmStatSevr";
  /* alarm.h from EPICS base define these enums:
     epicsAlarmCondition epicsAlarmSeverity
  but we use "int" here */
  const char *paramName = NULL;
  if (getParamName(axisNo, function, &paramName)) paramName = "";

  int oldStat = -1;
  int oldSevr = -1;
  int newStat = STATE_ALARM; /* Assume the worst */
  int newSevr = INVALID_ALARM;
  getParamAlarmStatus(axisNo, function, &oldStat);
  getParamAlarmSeverity(axisNo, function, &oldSevr);
  switch (status) {
    case asynSuccess:
      newStat = NO_ALARM;
      newSevr = NO_ALARM;
      break;
    case asynTimeout:
    case asynOverflow:
    case asynError:
    case asynDisabled:
    default:
      newStat = STATE_ALARM; /* Assume the worst */
      newSevr = INVALID_ALARM;
      break;
    case asynDisconnected:
      newStat = COMM_ALARM;
      newSevr = INVALID_ALARM;
      break;
  }
  if (newStat != oldStat) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%s%s(%d) %s(%d) asynStatus=%s(%d) STAT=%s(%d)->%s(%d)\n",
              modNamEMC, functionName,
              axisNo, paramName, function,
              ethercatmcstrStatus(status), (int)status,
              epicsAlarmConditionStrings[oldStat], oldStat,
              epicsAlarmConditionStrings[newStat], newStat);
    setParamAlarmStatus(axisNo, function, newStat);
  }
  if (newSevr != oldSevr) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%s%s(%d) %s(%d) asynStatus=%s(%d) STAT=%s(%d)->%s(%d)\n",
              modNamEMC, functionName,
              axisNo, paramName, function,
              ethercatmcstrStatus(status), (int)status,
              epicsAlarmSeverityStrings[oldSevr], oldSevr,
              epicsAlarmSeverityStrings[newSevr], newSevr);
    setParamAlarmSeverity(axisNo, function, newSevr);
  }
}
