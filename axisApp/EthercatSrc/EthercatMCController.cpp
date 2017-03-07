/*
FILENAME... EthercatMCController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include "EthercatMC.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

const static char *const strEthercatMCCreateController = "EthercatMCCreateController";
const static char *const strEthercatMCConfigController = "EthercatMCConfigController";
const static char *const strEthercatMCReadController   = "EthercatMCReadController";
const static char *const strEthercatMCCreateAxisDef    = "EthercatMCCreateAxis";
const static char *const strCtrlReset = ".ctrl.ErrRst";

/** Creates a new EthercatMCController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the EthercatMC controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
EthercatMCController::EthercatMCController(const char *portName, const char *MotorPortName, int numAxes,
                                               double movingPollPeriod,double idlePollPeriod)
  :  asynAxisController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;

  /* Controller */
  memset(&ctrlLocal, 0, sizeof(ctrlLocal));
  createParam(EthercatMCMCUErrMsgString,     asynParamOctet,       &EthercatMCMCUErrMsg_);

  /* Per axis */
  createParam(EthercatMCErrString,           asynParamInt32,       &EthercatMCErr_);
  createParam(EthercatMCErrIdString,         asynParamInt32,       &EthercatMCErrId_);

  createParam(EthercatMCProcHomString,       asynParamInt32,       &EthercatMCProcHom_);
  createParam(EthercatMCErrRstString,        asynParamInt32,       &EthercatMCErrRst_);
  createParam(EthercatMCPosHomString,        asynParamFloat64,     &EthercatMCPosHom_);
  createParam(EthercatMCVelToHomString,      asynParamFloat64,     &EthercatMCVelToHom_);
  createParam(EthercatMCVelFrmHomString,     asynParamFloat64,     &EthercatMCVelFrmHom_);
  createParam(EthercatMCAccHomString,        asynParamFloat64,     &EthercatMCAccHom_);
  createParam(EthercatMCDecHomString,        asynParamFloat64,     &EthercatMCDecHom_);
  createParam(EthercatMCVelActString,        asynParamFloat64,     &EthercatMCVelAct_);
  createParam(EthercatMCAccActString,        asynParamFloat64,     &EthercatMCAccAct_);
  createParam(EthercatMCDecActString,        asynParamFloat64,     &EthercatMCDecAct_);

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynAxisController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);
#endif

  /* Connect to EthercatMC controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "cannot connect to motor controller\n");
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new EthercatMCController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the EthercatMC controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int EthercatMCCreateController(const char *portName, const char *MotorPortName, int numAxes,
                                            int movingPollPeriod, int idlePollPeriod)
{
  new EthercatMCController(portName, MotorPortName, 1+numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
extern "C" int EthercatMCConfigController(int needOk, const char *portName,
                                          const char *configStr)
{
  EthercatMCController *pC;

  if (!portName || !configStr)  {
    printf("%sNULL parameter\n", strEthercatMCConfigController);
    return asynError;
  }
  pC = (EthercatMCController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           __FILE__, __FUNCTION__, portName);
    return asynError;
  }
  return pC->configController(needOk, configStr);
}

asynStatus EthercatMCController::configController(int needOk, const char *value)
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
           portName, strEthercatMCConfigController, portName, strCtrlReset);
    return asynError;
  }

  status = writeReadOnErrorDisconnect_C(pasynUserController_,
                                        value, configStrLen,
                                        inString, sizeof(inString));
  inString[sizeof(inString) -1] = '\0';
  if (status) {
    ctrlLocal.isConnected = 0;
  } else if (needOk) {
    status = checkACK(value, configStrLen, inString);
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "out=%s in=\"%s\" return=%s (%d)\n",
                value, inString,
                pasynManager->strStatus(status), (int)status);
      ctrlLocal.hasConfigError = 1;
      (void)setMCUErrMsg(inString);
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_INFO,
                "out=%s in=\"%s\"\n",
                value, inString);
    }
  } /* neddOK */

  printf("%s\n", inString);
  return status;
}

extern "C"
asynStatus writeReadOnErrorDisconnect_C(asynUser *pasynUser,
                                        const char *outdata, size_t outlen,
                                        char *indata, size_t inlen)
{
  size_t nwrite;
  asynStatus status = asynError;
  int eomReason;
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUser, outdata, outlen,
                                       indata, inlen,
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) ||
      (!nread && (eomReason & ASYN_EOM_END)))

{
#if 1
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
                  "out=%s status=%s (%d)\n",
                  outdata, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
#endif
    asynPrint(pasynUser, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s nread=%lu status=%s (%d)\n",
              outdata,(unsigned long)nread,
              pasynManager->strStatus(status), status);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus EthercatMCController::writeReadOnErrorDisconnect(void)
{
  asynStatus status = asynError;
  size_t outlen = strlen(outString_);
  inString_[0] = '\0';
  status = writeReadOnErrorDisconnect_C(pasynUserController_, outString_, outlen,
                                        inString_, sizeof(inString_));
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

asynStatus EthercatMCController::setMCUErrMsg(const char *value)
{
  asynStatus status = setStringParam(EthercatMCMCUErrMsg_, value);
  if (!status) status = callParamCallbacks();
  return status;
}

void EthercatMCController::handleStatusChange(asynStatus status)
{

  if (status && ctrlLocal.isConnected) {
    /* Connected -> Disconnected */
    int i;
    ctrlLocal.isConnected = 0;
    setMCUErrMsg("MCU Disconnected");
    for (i=0; i<numAxes_; i++) {
      EthercatMCAxis *pAxis=getAxis(i);
      if (!pAxis) continue;
      pAxis->handleDisconnect();
    }
  } else if (!status && !ctrlLocal.isConnected) {
    /* Disconnected -> Connected */
    ctrlLocal.isConnected = 1;
    setMCUErrMsg("MCU Cconnected");
  }
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynAxisController::report()
  */
void EthercatMCController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynAxisController::report(fp, level);
}

/** Returns a pointer to an EthercatMCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
EthercatMCAxis* EthercatMCController::getAxis(asynUser *pasynUser)
{
  return static_cast<EthercatMCAxis*>(asynAxisController::getAxis(pasynUser));
}

/** Returns a pointer to an EthercatMCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
EthercatMCAxis* EthercatMCController::getAxis(int axisNo)
{
  return static_cast<EthercatMCAxis*>(asynAxisController::getAxis(axisNo));
}


/** Code for iocsh registration */
static const iocshArg EthercatMCCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg EthercatMCCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg EthercatMCCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg EthercatMCCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg EthercatMCCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const EthercatMCCreateControllerArgs[] = {&EthercatMCCreateControllerArg0,
                                                            &EthercatMCCreateControllerArg1,
                                                            &EthercatMCCreateControllerArg2,
                                                            &EthercatMCCreateControllerArg3,
                                                            &EthercatMCCreateControllerArg4};
static const iocshFuncDef EthercatMCCreateControllerDef = {strEthercatMCCreateController, 5,
                                                           EthercatMCCreateControllerArgs};
static void EthercatMCCreateContollerCallFunc(const iocshArgBuf *args)
{
  EthercatMCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* EthercatMCConfigController */
static const iocshArg EthercatMCConfigControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg EthercatMCConfigControllerArg1 = {"ConfigString",         iocshArgString};
static const iocshArg * const EthercatMCConfigControllerArgs[] = {&EthercatMCConfigControllerArg0,
                                                                  &EthercatMCConfigControllerArg1};
static const iocshFuncDef EthercatMCConfigControllerDef = {strEthercatMCConfigController, 2,
                                                           EthercatMCConfigControllerArgs};
static const iocshFuncDef EthercatMCReadControllerDef = {strEthercatMCReadController, 2,
                                                         EthercatMCConfigControllerArgs};

static void EthercatMCConfigContollerCallFunc(const iocshArgBuf *args)
{
  int needOk = 1;
  EthercatMCConfigController(needOk, args[0].sval, args[1].sval);
}

static void EthercatMCReadContollerCallFunc(const iocshArgBuf *args)
{
  int needOk = 0;
  EthercatMCConfigController(needOk, args[0].sval, args[1].sval);
}

/* EthercatMCCreateAxis */
static const iocshArg EthercatMCCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg EthercatMCCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg EthercatMCCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg EthercatMCCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const EthercatMCCreateAxisArgs[] = {&EthercatMCCreateAxisArg0,
                                                            &EthercatMCCreateAxisArg1,
                                                            &EthercatMCCreateAxisArg2,
                                                            &EthercatMCCreateAxisArg3};
static const iocshFuncDef EthercatMCCreateAxisDef = {strEthercatMCCreateAxisDef, 4,
                                                     EthercatMCCreateAxisArgs};
static void EthercatMCCreateAxisCallFunc(const iocshArgBuf *args)
{
  EthercatMCCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void EthercatMCControllerRegister(void)
{
  iocshRegister(&EthercatMCCreateControllerDef, EthercatMCCreateContollerCallFunc);
  iocshRegister(&EthercatMCConfigControllerDef, EthercatMCConfigContollerCallFunc);
  iocshRegister(&EthercatMCReadControllerDef,   EthercatMCReadContollerCallFunc);
  iocshRegister(&EthercatMCCreateAxisDef,       EthercatMCCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(EthercatMCControllerRegister);
}
