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
    return "Amplifier off";
  case 0x4263:
    return "Is still proc";
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
    return "Follow err pos";
  case 0x4551:
    return "Follow err vel";
  case 0x4650:
    return "Drv HW not rdy";
  case 0x4655:
    return "Inv IO data";
  case 0x4B09:
    return "Axis not ready";
  case 0x4B0A:
    return "Homing failed";
  default:
    return "";
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
ethercatmcController::ethercatmcController(const char *portName,
                                           const char *MotorPortName, int numAxes,
                                           double movingPollPeriod,
                                           double idlePollPeriod,
                                           const char *optionStr)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
                         asynInt64Mask, // additional callback interface beyond those in base class
                         asynInt64Mask, // additional callback interface beyond those in base class
#else
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
#endif
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;

  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  ctrlLocal.oldStatus = asynDisconnected;
  ctrlLocal.cntADSstatus = 0;
  features_ = 0;
  lockADSsocket_ = epicsMutexMustCreate();
#ifndef motorMessageTextString
  createParam("MOTOR_MESSAGE_TEXT",          asynParamOctet,       &ethercatmcMCUErrMsg_);
#else
  createParam(ethercatmcMCUErrMsgString,     asynParamOctet,       &ethercatmcMCUErrMsg_);
#endif
  createParam(ethercatmcDbgStrToMcuString,   asynParamOctet,       &ethercatmcDbgStrToMcu_);
  createParam(ethercatmcDbgStrToLogString,   asynParamOctet,       &ethercatmcDbgStrToLog_);

  /* Per axis */
  createParam(ethercatmcErrString,           asynParamInt32,       &ethercatmcErr_);
  createParam(ethercatmcErrIdString,         asynParamInt32,       &ethercatmcErrId_);

  createParam(ethercatmcEnc_ActString,       asynParamFloat64,     &ethercatmcEncAct_);
  createParam(ethercatmcHomProcString,       asynParamInt32,       &ethercatmcHomProc_);
  createParam(ethercatmcHomPosString,        asynParamFloat64,     &ethercatmcHomPos_);
  createParam(ethercatmcStatusCodeString,    asynParamInt32,       &ethercatmcStatusCode_);
  createParam(ethercatmcStatusBitsString,    asynParamInt32,       &ethercatmcStatusBits_);
  createParam(ethercatmcFoffVisString,       asynParamInt32,       &ethercatmcFoffVis_);
  createParam(ethercatmcHomProc_RBString,    asynParamInt32,       &ethercatmcHomProc_RB_);
  createParam(ethercatmcHomPos_RBString,     asynParamFloat64,     &ethercatmcHomPos_RB_);
  createParam(ethercatmcVelToHomString,      asynParamFloat64,     &ethercatmcVelToHom_);
  createParam(ethercatmcVelFrmHomString,     asynParamFloat64,     &ethercatmcVelFrmHom_);
  createParam(ethercatmcAccHomString,        asynParamFloat64,     &ethercatmcAccHom_);
  createParam(ethercatmcErrRstString,        asynParamInt32,       &ethercatmcErrRst_);
  createParam(ethercatmcVelActString,        asynParamFloat64,     &ethercatmcVelAct_);
  createParam(ethercatmcVel_RBString,        asynParamFloat64,     &ethercatmcVel_RB_);
  createParam(ethercatmcAcc_RBString,        asynParamFloat64,     &ethercatmcAcc_RB_);
  createParam(ethercatmcNamAux0_String,      asynParamOctet,       &ethercatmcNamAux0_);
  createParam(ethercatmcNamAux1_String,      asynParamOctet,       &ethercatmcNamAux1_);
  createParam(ethercatmcNamAux2_String,      asynParamOctet,       &ethercatmcNamAux2_);
  createParam(ethercatmcNamAux3_String,      asynParamOctet,       &ethercatmcNamAux3_);
  createParam(ethercatmcNamAux4_String,      asynParamOctet,       &ethercatmcNamAux4_);
  createParam(ethercatmcNamAux5_String,      asynParamOctet,       &ethercatmcNamAux5_);
  createParam(ethercatmcNamAux6_String,      asynParamOctet,       &ethercatmcNamAux6_);
  createParam(ethercatmcNamAux7_String,      asynParamOctet,       &ethercatmcNamAux7_);
  createParam(ethercatmcNamBit24_String,     asynParamOctet,       &ethercatmcNamBit24_);
  createParam(ethercatmcNamBit25_String,     asynParamOctet,       &ethercatmcNamBit25_);
  createParam(ethercatmcCfgVELO_String,      asynParamFloat64,     &ethercatmcCfgVELO_);
  createParam(ethercatmcCfgVMAX_String,      asynParamFloat64,     &ethercatmcCfgVMAX_);
  createParam(ethercatmcCfgJVEL_String,      asynParamFloat64,     &ethercatmcCfgJVEL_);
  createParam(ethercatmcCfgACCS_String,      asynParamFloat64,     &ethercatmcCfgACCS_);
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
    /*  Find additional devices/asynParams */
    poll();
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
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

void ethercatmcController::handleStatusChange(asynStatus status)
{
  if (status != ctrlLocal.oldStatus) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%soldStatus=%s (%d) status=%s (%d)\n",
              modNamEMC,
              ethercatmcstrStatus(ctrlLocal.oldStatus), (int)ctrlLocal.oldStatus,
              ethercatmcstrStatus(status), (int)status);
    if (status) {
      /* Connected -> Disconnected */
      int i;
      ctrlLocal.initialPollDone = 0;
      /* Keep bits that are specified via options,
         clear bits that are fetched from the controller */
      features_ &= ~reportedFeatureBits;
      setMCUErrMsg("MCU Disconnected");
      for (i=0; i<numAxes_; i++) {
        asynMotorAxis *pAxis=getAxis(i);
        if (!pAxis) continue;
        pAxis->setIntegerParam(motorStatusCommsError_, 1);
        pAxis->callParamCallbacks();
      }
      free(ctrlLocal.pIndexerProcessImage);
      ctrlLocal.pIndexerProcessImage = NULL;
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
      status = initialPollIndexer();
      if (!status) {
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
      return pollIndexer();
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
  asynStatus status = asynSuccess;
  asynMotorAxis *pAxis;
  int function = pasynUser->reason;

  pAxis = getAxis(pasynUser);
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

static void ethercatmcCreateAxisCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ethercatmcCreateIndexerAxisCallFunc(const iocshArgBuf *args)
{
  ethercatmcCreateIndexerAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void ethercatmcControllerRegister(void)
{
  iocshRegister(&ethercatmcCreateControllerDef, ethercatmcCreateContollerCallFunc);
  iocshRegister(&ethercatmcConfigOrDieDef,      ethercatmcConfigOrDieCallFunc);
  iocshRegister(&ethercatmcConfigControllerDef, ethercatmcConfigContollerCallFunc);
  iocshRegister(&ethercatmcReadControllerDef,   ethercatmcReadContollerCallFunc);
  iocshRegister(&ethercatmcCreateAxisDef,       ethercatmcCreateAxisCallFunc);
  iocshRegister(&ethercatmcCreateIndexerAxisDef,ethercatmcCreateIndexerAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(ethercatmcControllerRegister);
}
