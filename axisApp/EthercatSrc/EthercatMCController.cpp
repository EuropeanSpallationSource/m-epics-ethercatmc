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

  createParam(EthercatMCErrString,           asynParamInt32,       &EthercatMCErr_);
  createParam(EthercatMCErrIdString,         asynParamInt32,       &EthercatMCErrId_);
  createParam(EthercatMCErrMsgString,        asynParamOctet,       &EthercatMCErrMsg_);

  createParam(EthercatMCProcHomString,       asynParamInt32,       &EthercatMCProcHom_);
  createParam(EthercatMCErrRstString,        asynParamInt32,       &EthercatMCErrRst_);
  createParam(EthercatMCJVELString,          asynParamFloat64,     &EthercatMCJVEL_);

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
asynStatus EthercatMCController::writeReadOnErrorDisconnect(void)
{
  size_t nwrite = 0;
  asynStatus status;
  int eomReason;
  size_t outlen = strlen(outString_);
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUserController_, outString_, outlen,
                                       inString_, sizeof(inString_),
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) ||
      (!nread && (eomReason & ASYN_EOM_END)))

{
#if 1
    asynInterface *pasynInterface = NULL;
    asynCommon     *pasynCommon = NULL;
    pasynInterface = pasynManager->findInterface(pasynUserController_,
                                                 asynCommonType,
                                                 0 /* FALSE */);
    if (pasynInterface) {
      pasynCommon = (asynCommon *)pasynInterface->pinterface;
      status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUserController_);
      if (status != asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s status=%s (%d)\n",
                  outString_, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
#endif
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s nread=%lu status=%s (%d)\n",
              outString_,(unsigned long)nread,
              pasynManager->strStatus(status), status);
    handleStatusChange(asynError);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

void EthercatMCController::handleStatusChange(asynStatus status)
{
  int i;
  for (i=0; i<numAxes_; i++) {
    EthercatMCAxis *pAxis=getAxis(i);
    if (!pAxis) continue;
    pAxis->handleDisconnect();
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


asynStatus EthercatMCController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  EthercatMCAxis *pAxis;
  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  (void)pAxis->setIntegerParam(function, value);
  return asynAxisController::writeInt32(pasynUser, value);
}

/** Code for iocsh registration */
static const iocshArg EthercatMCCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg EthercatMCCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg EthercatMCCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg EthercatMCCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg EthercatMCCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const EthercatMCCreateControllerArgs[] = {&EthercatMCCreateControllerArg0,
                                                             &EthercatMCCreateControllerArg1,
                                                             &EthercatMCCreateControllerArg2,
                                                             &EthercatMCCreateControllerArg3,
                                                             &EthercatMCCreateControllerArg4};
static const iocshFuncDef EthercatMCCreateControllerDef = {"EthercatMCCreateController", 5, EthercatMCCreateControllerArgs};
static void EthercatMCCreateContollerCallFunc(const iocshArgBuf *args)
{
  EthercatMCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
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
static const iocshFuncDef EthercatMCCreateAxisDef = {"EthercatMCCreateAxis", 4, EthercatMCCreateAxisArgs};
static void EthercatMCCreateAxisCallFunc(const iocshArgBuf *args)
{
  EthercatMCCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void EthercatMCControllerRegister(void)
{
  iocshRegister(&EthercatMCCreateControllerDef, EthercatMCCreateContollerCallFunc);
  iocshRegister(&EthercatMCCreateAxisDef,       EthercatMCCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(EthercatMCControllerRegister);
}
