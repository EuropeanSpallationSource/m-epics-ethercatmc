/*
  FILENAME... ethercatmcIndexerAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "motor.h"
#include "ethercatmcController.h"
#include "ethercatmcIndexerAxis.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#ifndef ASYN_TRACE_DEBUG
#define ASYN_TRACE_DEBUG     0x0080
#endif


extern "C" const char *idxStatusCodeTypeToStr(idxStatusCodeType idxStatusCode)
{
  switch (idxStatusCode) {
  case  idxStatusCodeRESET:    return "RSET";
  case  idxStatusCodeIDLE:     return "IDLE";
  case  idxStatusCodePOWEROFF: return "POFF";
  case  idxStatusCodeWARN:     return "WARN";
  case  idxStatusCodeERR4:     return "ERR4";
  case  idxStatusCodeSTART:    return "STRT";
  case  idxStatusCodeBUSY:     return "BUSY";
  case  idxStatusCodeSTOP:     return "STOP";
  case  idxStatusCodeERROR:    return "ERRO";
  case  idxStatusCodeERR9:     return "ERR9";
  case  idxStatusCodeERR10:    return "ERRA";
  case  idxStatusCodeERR11:    return "ERRB";
  case  idxStatusCodeERR12:    return "ERRC";
  case  idxStatusCodeERR13:    return "ERRD";
  case  idxStatusCodeERR14:    return "ERRE";
  case  idxStatusCodeERR15:    return "ERRF";
  default:                     return "UKWN";
  }
}


//
// These are the ethercatmcIndexerAxis methods
//

/** Creates a new ethercatmcIndexerAxis object.
 * \param[in] pC Pointer to the ethercatmcController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_.
 * (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
ethercatmcIndexerAxis::ethercatmcIndexerAxis(ethercatmcController *pC,
                                             int axisNo,
                                             int axisFlags,
                                             const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
#ifdef motorFlagsDriverUsesEGUString
  setIntegerParam(pC_->motorFlagsDriverUsesEGU_,1);
#endif
#ifdef motorFlagsAdjAfterHomedString
  setIntegerParam(pC_->motorFlagsAdjAfterHomed_, 1);
#endif
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  /* We pretend to have an encoder (fActPosition) */
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
#ifdef motorFlagsNoStopProblemString
  setIntegerParam(pC_->motorFlagsNoStopProblem_, 1);
#endif
#ifdef motorFlagsNoStopOnLsString
  setIntegerParam(pC_->motorFlagsNoStopOnLS_, 1);
#endif
#ifdef motorFlagsLSrampDownString
  setIntegerParam(pC_->motorFlagsLSrampDown_, 1);
#endif
#ifdef motorFlagsPwrWaitForOnString
  setIntegerParam(pC_->motorFlagsPwrWaitForOn_, 1);
#endif

#ifdef motorShowPowerOffString
  setIntegerParam(pC_->motorShowPowerOff_, 1);
#endif
#ifdef motorFlagsHomeOnLsString
  setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);
#endif

#ifdef motorFlagsNotHomedProblemString
  setIntegerParam(pC_->motorFlagsNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
#ifdef  motorNotHomedProblemString
  setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
  setStringParam(pC_->defAsynPara.ethercatmcNamBit25_, "Dynamic_problem_timeout");
  setStringParam(pC_->defAsynPara.ethercatmcNamBit24_, "Static_problem_inhibit");

  /* Set the module name to "" if we have FILE/LINE enabled by asyn */
  if (pasynTrace->getTraceInfoMask(pC_->pasynUserController_) &
      ASYN_TRACEINFO_SOURCE) {
    modNamEMC = "";
  }
}

extern "C" int ethercatmcCreateIndexerAxis(const char *ethercatmcName,
                                           int axisNo,
                                           int axisFlags,
                                           const char *axisOptionsStr)
{
  ethercatmcController *pC;
  /* parameters not used */
  (void)axisFlags;
  (void)axisOptionsStr;

  pC = (ethercatmcController*) findAsynPortDriver(ethercatmcName);
  if (!pC) {
    printf("Error port %s not found\n", ethercatmcName);
    return asynError;
  }
  /* The indexer may have created an axis himself, when polling the controller.
     However, if there is a network problem, ADS does not work
     or any other reason that the indexer can not reach the controller,
     create one here to have the records showing "Communication Error" */
  pC->lock();
  if (!pC->getAxis(axisNo)) {
    new ethercatmcIndexerAxis(pC, axisNo, axisFlags, axisOptionsStr);
  }
  pC->unlock();
  //ethercatmcIndexerAxis *pAxis =
  //  static_cast<ethercatmcIndexerAxis*>(pC->asynMotorController::getAxis(axisNo));
  if (axisOptionsStr && axisOptionsStr[0]) {

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (0) {
        ;
      } else {
        printf("Error option \"%s\" not supported\n", pThisOption);
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
  if (axisFlags & AMPLIFIER_ON_FLAG_AUTO_ON) {
    asynMotorAxis *pAxis = pC->getAxis(axisNo);
    if (pAxis) {
      asynStatus status;
      int function;
      status = pC->findParam(motorPowerAutoOnOffString, &function);
      if (!status) {
#ifdef POWERAUTOONOFFMODE2
        pAxis->setIntegerParam(function, POWERAUTOONOFFMODE2);
#else
        pAxis->setIntegerParam(function, 1);
#endif
      }
      status = pC->findParam(motorPowerOnDelayString, &function);
      if (!status) {
        pAxis->setDoubleParam(function, 6.0);
      }
      status = pC->findParam(motorPowerOffDelayString, &function);
      if (!status) {
        pAxis->setDoubleParam(function, -1.0);
      }
    }
  }
  return asynSuccess;
}

void ethercatmcIndexerAxis::setIndexerDevNumOffsetTypeCode(unsigned devNum,
                                                           unsigned iOffset,
                                                           unsigned iTypCode)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sOffsetTypeCode(%d) devNum=%u iTypCode=0x%X, iOffset=%u\n",
            modNamEMC, axisNo_, devNum, iTypCode, iOffset);
  drvlocal.devNum = devNum;
  drvlocal.iTypCode = iTypCode;
  drvlocal.iOffset = iOffset;
  if ((drvlocal.iTypCode == 0x5008) || (drvlocal.iTypCode == 0x500c)) {
    drvlocal.lenInPlcPara = 4;
    drvlocal.paramIfOffset = drvlocal.iOffset + 0xA;
  } else if (drvlocal.iTypCode == 0x5010) {
    drvlocal.lenInPlcPara = 8;
    drvlocal.paramIfOffset = drvlocal.iOffset + 22;
  } else if (drvlocal.iTypCode == 0x1802) {
    drvlocal.lenInPlcPara = 0;
    drvlocal.paramIfOffset = 0;
  } else if (drvlocal.iTypCode == 0x1E04) {
    drvlocal.lenInPlcPara = 2;
    drvlocal.paramIfOffset = 0;
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d) iTypCode=0x%X\n",
              modNamEMC, axisNo_, drvlocal.iTypCode);
  }
#ifdef motorFlagsNtmUpdateString
  if (pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset) {
    setIntegerParamLog(pC_-> motorFlagsNtmUpdate_, 1, "NtmUpdate");
  }
#endif
}

void ethercatmcIndexerAxis::setAuxBitsNotHomedMask(unsigned auxBitsNotHomedMask)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d) auxBitsNotHomedMask=0x%X\n",
            modNamEMC, axisNo_, auxBitsNotHomedMask);
  drvlocal.auxBitsNotHomedMask = auxBitsNotHomedMask;
}

void ethercatmcIndexerAxis::setAuxBitsEnabledMask(unsigned auxBitsEnabledMask)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d) setAuxBitsEnabledMask=0x%X\n",
            modNamEMC, axisNo_, auxBitsEnabledMask);
  drvlocal.auxBitsEnabledMask = auxBitsEnabledMask;
}

void ethercatmcIndexerAxis::setAuxBitsLocalModeMask(unsigned auxBitsLocalModeMask)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d) setAuxBitsLocalModeMask=0x%X\n",
            modNamEMC, axisNo_, auxBitsLocalModeMask);
  drvlocal.auxBitsLocalModeMask = auxBitsLocalModeMask;
}

void ethercatmcIndexerAxis::setAuxBitsHomeSwitchMask(unsigned auxBitsHomeSwitchMask)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d) setAuxBitsHomeswitchMask=0x%X\n",
            modNamEMC, axisNo_, auxBitsHomeSwitchMask);
  drvlocal.auxBitsHomeSwitchMask = auxBitsHomeSwitchMask;
}

void ethercatmcIndexerAxis::addPollNowParam(uint8_t paramIndex)
{
  size_t pollNowIdx;
  const size_t pollNowIdxMax = sizeof(drvlocal.pollNowParams)/sizeof(drvlocal.pollNowParams[0]) - 1;
  for (pollNowIdx = 0; pollNowIdx < pollNowIdxMax; pollNowIdx++) {
    if (drvlocal.pollNowParams[pollNowIdx] == paramIndex) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%saddPollNowParam(%d) paramIndex=%s (%d) already at pollNowIdx=%u\n",
                modNamEMC, axisNo_,
                plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                (unsigned)pollNowIdx);
      return;
    }
    if (!drvlocal.pollNowParams[pollNowIdx]) {
      drvlocal.pollNowParams[pollNowIdx] = paramIndex;
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%saddPollNowParam(%d) paramIndex=%s (%d) new at pollNowIdx=%u\n",
                modNamEMC, axisNo_,
                plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                (unsigned)pollNowIdx);
      return;
    }
  }
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void ethercatmcIndexerAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}




/** Move the axis to a position, either absolute or relative
 * \param[in] position in steps
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, steps/sec
 * \param[in] maximum velocity, steps/sec
 * \param[in] acceleration,  steps/sec/sec
 *
 */
asynStatus ethercatmcIndexerAxis::move(double position, int relative,
                                       double minVelocity, double maxVelocity,
                                       double acceleration)
{
  asynStatus status = asynSuccess;
  unsigned traceMask = ASYN_TRACE_INFO;

  asynPrint(pC_->pasynUserController_, traceMask,
            "%smove (%d) position=%f relative=%d minVelocity=%f maxVelocity=%f"
            " acceleration=%f\n",
            "ethercatmcIndexerAxis", axisNo_,
            position, relative, minVelocity, maxVelocity, acceleration);

  if (drvlocal.paramIfOffset) {
    if ((maxVelocity > 0.0)  &&
        (drvlocal.PILSparamPerm[PARAM_IDX_SPEED_FLOAT] == PILSparamPermWrite)) {
      double oldValue, valueRB;
      status = pC_->getDoubleParam(axisNo_, pC_->defAsynPara.ethercatmcVel_RB_, &oldValue);
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
                "%smove (%d) ethercatmcVel status=%s (%d)\n",
                "ethercatmcIndexerAxis", axisNo_,
                ethercatmcstrStatus(status), (int)status);
      if ((status != asynParamUndefined) && (maxVelocity != oldValue)) {
        status = pC_->indexerParamWrite(this, PARAM_IDX_SPEED_FLOAT,
                                        maxVelocity, &valueRB);
        if (status) {
          asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                    "%smove (%d) status=%s (%d)\n",
                    "ethercatmcIndexerAxis", axisNo_,
                    ethercatmcstrStatus(status), (int)status);
          if (status != asynParamWrongType) {
            return status;
          }
        } else {
          setDoubleParam(pC_->defAsynPara.ethercatmcVel_RB_, maxVelocity);
        }
      }
    }
    if ((acceleration > 0.0) &&
        (drvlocal.PILSparamPerm[PARAM_IDX_ACCEL_FLOAT] == PILSparamPermWrite)) {
      double oldValue, valueRB;
      status = pC_->getDoubleParam(axisNo_, pC_->defAsynPara.ethercatmcAcc_RB_, &oldValue);
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
                "%smove (%d) ethercatmcAcc status=%s (%d)\n",
                "ethercatmcIndexerAxis", axisNo_,
                ethercatmcstrStatus(status), (int)status);
      if ((status != asynParamUndefined) && (acceleration != oldValue)) {
        status = pC_->indexerParamWrite(this, PARAM_IDX_ACCEL_FLOAT,
                                        acceleration, &valueRB);
        if (status) {
          asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                    "%smove (%d) status=%s (%d)\n",
                    "ethercatmcIndexerAxis", axisNo_,
                    ethercatmcstrStatus(status), (int)status);
          if (status != asynParamWrongType) {
            return status;
          }
        } else {
          setDoubleParam(pC_->defAsynPara.ethercatmcAcc_RB_, acceleration);
        }
      }
    }
  }
  if (relative && pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset) {
    /* Talking to the simulatar. test case 141 needs a "real"
       moveRelative to be commanded to the controller. */
    char buf[40];
    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf) -1,
             "Sim.this.moveRelative=%f",
             position);
    return setStringParamDbgStrToMcu(buf);
  } else if (relative){
    double actPosition;
    pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &actPosition);
    position = position - actPosition;
  }
  if ((drvlocal.iTypCode == 0x5008) || (drvlocal.iTypCode == 0x500c)) {
    unsigned cmdReason = idxStatusCodeSTART  << 12;
    struct {
      uint8_t  posRaw[4];
      uint8_t  cmdReason[2];
    } posCmd;
    DOUBLETONET(position, posCmd.posRaw);
    UINTTONET(cmdReason, posCmd.cmdReason);
    return pC_->setPlcMemoryViaADS(drvlocal.iOffset + drvlocal.lenInPlcPara,
                                   &posCmd, sizeof(posCmd));
  } else if (drvlocal.iTypCode == 0x5010) {
    unsigned cmdReason = idxStatusCodeSTART  << (12 + 16);
    struct {
      uint8_t  posRaw[8];
      uint8_t  cmdReason[4];
    } posCmd;
    DOUBLETONET(position, posCmd.posRaw);
    UINTTONET(cmdReason, posCmd.cmdReason);
    return pC_->setPlcMemoryViaADS(drvlocal.iOffset + drvlocal.lenInPlcPara,
                                   &posCmd, sizeof(posCmd));
  } else if (drvlocal.iTypCode == 0x1E04) {
    unsigned cmdReason = idxStatusCodeSTART  << (12 + 16);
    struct {
      uint8_t  targetValueRaw[2];
      uint8_t  cmdReason[4];
    } targetCmd;
    int iPosition = (short)position;
    UINTTONET(iPosition, targetCmd.targetValueRaw);
    UINTTONET(cmdReason, targetCmd.cmdReason);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%smove(%d) iTypCode=0x%X iPosition=%i cmdReason=0x%08X drvlocal.iOffset=%u drvlocal.lenInPlcPara=%u\n",
              modNamEMC, axisNo_, drvlocal.iTypCode,
              iPosition, cmdReason,
              drvlocal.iOffset,
              drvlocal.lenInPlcPara);
    return pC_->setPlcMemoryViaADS(drvlocal.iOffset + drvlocal.lenInPlcPara,
                                   &targetCmd, sizeof(targetCmd));
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%smove(%d) iTypCode=0x%X\n",
            modNamEMC, axisNo_, drvlocal.iTypCode);
  return asynError;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus ethercatmcIndexerAxis::home(double minVelocity, double maxVelocity,
                                       double acceleration, int forwards)
{
  asynStatus status;
  (void)minVelocity;
  (void)maxVelocity;
  (void)acceleration;
  (void)forwards;
  status = pC_->indexerParamWrite(this, PARAM_IDX_FUN_REFERENCE,
                                  0.0, NULL);
  return status;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus ethercatmcIndexerAxis::moveVelocity(double minVelocity,
                                               double maxVelocity,
                                               double acceleration)
{
  unsigned traceMask = ASYN_TRACE_INFO;
  asynStatus status;
  double veloRB = 0.0;
  int motorStatusDone = 0;
  (void)minVelocity;
  (void)acceleration;

  pC_->getIntegerParam(axisNo_, pC_->motorStatusDone_, &motorStatusDone);
  asynPrint(pC_->pasynUserController_, traceMask,
            "%smoveVelocity (%d) minVelocity=%f maxVelocity=%f"
            " acceleration=%f motorStatusDone=%d\n", modNamEMC, axisNo_,
            minVelocity, maxVelocity, acceleration, motorStatusDone);

  if (!motorStatusDone) {
    stopAxisInternal("moveVelocity", acceleration);
  }
  if ((acceleration > 0.0) &&
      (drvlocal.PILSparamPerm[PARAM_IDX_ACCEL_FLOAT] == PILSparamPermWrite)) {
    double oldValue, valueRB;
    pC_->getDoubleParam(axisNo_, pC_->defAsynPara.ethercatmcAcc_RB_, &oldValue);
    if (acceleration != oldValue) {
      status = pC_->indexerParamWrite(this, PARAM_IDX_ACCEL_FLOAT,
                                      acceleration, &valueRB);
      if (status) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                  "%smoveVelocity (%d) status=%s (%d)\n",
                  "ethercatmcIndexerAxis", axisNo_,
                  ethercatmcstrStatus(status), (int)status);
        if (status != asynParamWrongType) {
          return status;
        }
      } else {
        setDoubleParam(pC_->defAsynPara.ethercatmcAcc_RB_, acceleration);
      }
    }
  }
  status = pC_->indexerParamWrite(this, PARAM_IDX_FUN_MOVE_VELOCITY,
                                  maxVelocity, &veloRB);
  asynPrint(pC_->pasynUserController_, traceMask,
            "%smoveVelocity (%d) veloRB=%f status=%s(%d)\n",
            "ethercatmcIndexerAxis", axisNo_,
            veloRB, ethercatmcstrStatus(status), (int)status);
  if (status) return status;
  setDoubleParam(pC_->defAsynPara.ethercatmcVel_RB_, veloRB);
  return status;
}


/**
 * See asynMotorAxis::setPosition
 */
asynStatus ethercatmcIndexerAxis::setPosition(double value)
{
  asynStatus status = asynError;
  if (drvlocal.paramIfOffset) {
    status = pC_->indexerParamWrite(this, PARAM_IDX_FUN_SET_POSITION,
                                    value, NULL);
  }
  return status;
}

asynStatus ethercatmcIndexerAxis::writeCmdRegisster(unsigned idxStatusCode)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%swriteCmdRegisster(%d) idxStatusCode=0x%X (%s)\n",
            modNamEMC, axisNo_, idxStatusCode,
            idxStatusCodeTypeToStr((idxStatusCodeType)idxStatusCode));
  if ((drvlocal.iTypCode == 0x5008) || (drvlocal.iTypCode == 0x500c)) {
    unsigned cmdReason = idxStatusCode << 12;
    struct {
      uint8_t  cmdReason[2];
    } posCmd;
    UINTTONET(cmdReason, posCmd.cmdReason);
    return pC_->setPlcMemoryViaADS(drvlocal.iOffset + (2 * drvlocal.lenInPlcPara),
                                   &posCmd, sizeof(posCmd));
  } else if (drvlocal.iTypCode == 0x5010 || drvlocal.iTypCode == 0x1E04) {
    unsigned cmdReason = idxStatusCode << (12 + 16);
    struct {
      uint8_t  cmdReason[4];
    } posCmd;
    UINTTONET(cmdReason, posCmd.cmdReason);
    return pC_->setPlcMemoryViaADS(drvlocal.iOffset + (2 * drvlocal.lenInPlcPara),
                                   &posCmd, sizeof(posCmd));
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%swriteCmdRegisster(%d) iTypCode=0x%X\n",
            modNamEMC, axisNo_, drvlocal.iTypCode);
  return asynError;
}

/** Stop the axis
 *
 */
asynStatus ethercatmcIndexerAxis::stopAxisInternal(const char *function_name,
                                                   double acceleration)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sstopAxisInternal(%d) (%s)\n",
            modNamEMC, axisNo_, function_name);

  return writeCmdRegisster(idxStatusCodeSTOP);
}

/** Stop the axis, called by motor Record
 *
 */
asynStatus ethercatmcIndexerAxis::stop(double acceleration )
{
  //drvlocal.eeAxisWarning = eeAxisWarningNoWarning;
  return stopAxisInternal(__FUNCTION__, acceleration);
}

asynStatus ethercatmcIndexerAxis::setIntegerParamLog(int function,
                                                     int newValue,
                                                     const char *name)
{
  int oldValue;
  asynStatus status = pC_->getIntegerParam(axisNo_, function, &oldValue);
  if (status || (newValue != oldValue)) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d) %s=%d\n",
              modNamEMC, axisNo_, name, newValue);
  }
  return setIntegerParam(function, newValue);
}

/**
 * Called from the poller.
 * Before this axis-related method is called, the controller has
 * read the process image, and getPlcMemoryFromProcessImage()
 * can be used.
 * At the it it calls callParamCallbacks()
 */
asynStatus ethercatmcIndexerAxis::poll(bool *moving)
{
  bool cached = true;
  asynStatus status = ethercatmcIndexerAxis::doThePoll(cached, moving);
  if (status) {
    return status;
  }
  callParamCallbacks();
  return status;
}


/** Polls the axis.
 * This function reads the motor position, the limit status, the home status,
 * the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls.
 * \param[out] moving A flag that is set indicating that the axis is moving
 * (true) or done (false). */
asynStatus ethercatmcIndexerAxis::doThePoll(bool cached, bool *moving)
{
  asynStatus status = asynSuccess;
  unsigned traceMask = ASYN_TRACE_INFO;
  const char *msgTxtFromDriver = NULL;
  //double targetPosition = 0.0;
  double actPosition = 0.0;
  double paramfValue = 0.0;
  unsigned statusReasonAux, paramCtrl = 0;
  int errorID = -1;
  bool nowMoving = false;
  int powerIsOn = 0;
  int statusValid = 0;
  int positionValid = 1; /* all states except RESET */
  int hasError = 0;
  int localMode = 0;
  idxStatusCodeType idxStatusCode;
  unsigned idxReasonBits = 0;
  unsigned idxAuxBits = 0;
  int homed = 0;

  /* Don't leave *moving un-initialized, if we return early */
  *moving = false;
  if (!drvlocal.iTypCode) {
    /* No axis, (may be dummy-axis 0), return */
    return asynSuccess;
  }
  if (!drvlocal.iOffset) return asynSuccess;

  if (drvlocal.dirty.initialPollNeeded) {
    switch (drvlocal.iTypCode) {
    case 0x1E04:
        readAuxBitNamesEnums();
        status = asynSuccess;
      break;
    case 0x5008:
    case 0x5010:
        status = pC_->indexerReadAxisParameters(this, drvlocal.devNum);
      break;
    default:
      status = asynSuccess;
      break;
    }
    if (!status) {
      drvlocal.dirty.initialPollNeeded = 0;
      setIntegerParam(pC_->motorStatusCommsError_, 0);
      pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcStatusBits_,
                                         asynSuccess);
      pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcStatusCode_,
                                         asynSuccess);
      pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcMcuErr_,
                                         asynSuccess);
      pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcErrId_,
                                         asynSuccess);
      updateMsgTxtFromDriver(NULL);
    }
  }
  pC_->getIntegerParam(axisNo_, pC_->motorStatusPowerOn_, &powerIsOn);

  if ((drvlocal.iTypCode == 0x5008) || (drvlocal.iTypCode == 0x500c)) {
    struct {
      uint8_t   actPos[4];
      uint8_t   targtPos[4];
      uint8_t   statReasAux[2];
      uint8_t   paramCtrl[2];
      uint8_t   paramValue[4];
    } readback;
    uint16_t statusReasonAux16;
    if (cached) {
      status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                                 &readback,
                                                 sizeof(readback));
    } else {
      status = pC_->getPlcMemoryViaADS(drvlocal.iOffset,
                                       &readback,
                                       sizeof(readback));
    }
    if (status) {
      return status;
    }
    actPosition = NETTODOUBLE(readback.actPos);
    //targetPosition = NETTODOUBLE(readback.targtPos);
    statusReasonAux16 = NETTOUINT(readback.statReasAux);
    paramCtrl = NETTOUINT(readback.paramCtrl);
    {
      unsigned paramIndex = paramCtrl & PARAM_IF_IDX_MASK;
      unsigned lenInPlcPara = 0;
      if (drvlocal.lenInPlcParaInteger[paramIndex]) {
        lenInPlcPara = drvlocal.lenInPlcParaInteger[paramIndex];
        paramfValue = (double)netToUint(&readback.paramValue, lenInPlcPara);
      } else if (drvlocal.lenInPlcParaFloat[paramIndex]) {
        lenInPlcPara = drvlocal.lenInPlcParaFloat[paramIndex];
        paramfValue = netToDouble(&readback.paramValue, lenInPlcPara);
      }
    }
    /* Specific bit positions for 5008 */
    idxStatusCode = (idxStatusCodeType)(statusReasonAux16 >> 12);
    idxReasonBits = (statusReasonAux16 >> 8) & 0x0F;
    idxAuxBits    =  statusReasonAux16  & 0x0FF;

    /* 8 aux bits */
    statusReasonAux = statusReasonAux16 & 0xFF;
    /* 4 reason bits */
    statusReasonAux |= (idxReasonBits << 24);
    /* The poller had read errorID as a device, the result
       is in the parameter library */
    pC_->getIntegerParam(axisNo_, pC_->defAsynPara.ethercatmcErrId_, &errorID);
  } else if (drvlocal.iTypCode == 0x5010) {
    struct {
      uint8_t   actPos[8];
      uint8_t   targtPos[8];
      uint8_t   statReasAux[4];
      uint8_t   errorID[2];
      uint8_t   paramCtrl[2];
      uint8_t   paramValue[8];
    } readback;
    if (cached) {
      status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                                 &readback,
                                                 sizeof(readback));
    } else {
      status = pC_->getPlcMemoryViaADS(drvlocal.iOffset,
                                       &readback,
                                       sizeof(readback));
    }
    if (status) {
      return status;
    }
    actPosition = NETTODOUBLE(readback.actPos);
    //targetPosition = NETTODOUBLE(readback.targtPos);
    statusReasonAux = NETTOUINT(readback.statReasAux);
    paramCtrl = NETTOUINT(readback.paramCtrl);
    {
      unsigned paramIndex = paramCtrl & PARAM_IF_IDX_MASK;
      unsigned lenInPlcPara = 0;
      if (drvlocal.lenInPlcParaInteger[paramIndex]) {
        lenInPlcPara = drvlocal.lenInPlcParaInteger[paramIndex];
        paramfValue = (double)netToUint(&readback.paramValue, lenInPlcPara);
      } else if (drvlocal.lenInPlcParaFloat[paramIndex]) {
        lenInPlcPara = drvlocal.lenInPlcParaFloat[paramIndex];
        paramfValue = netToDouble(&readback.paramValue, lenInPlcPara);
      }
    }

    /* Specific for 5010 */
    errorID = (int)NETTOUINT(readback.errorID);
    setIntegerParam(pC_->defAsynPara.ethercatmcErrId_, errorID);

    idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
    idxReasonBits = (statusReasonAux >> 24) & 0x0F;
    idxAuxBits    =  statusReasonAux  & 0x03FFFFFF;
  } else if (drvlocal.iTypCode == 0x1802) {
    struct {
      uint8_t   statReasAux[4];
    } readback;
    status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                               &readback,
                                               sizeof(readback));
    statusReasonAux = NETTOUINT(readback.statReasAux);
    idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
    idxReasonBits = (statusReasonAux >> 24) & 0x0F;
    idxAuxBits    =  statusReasonAux  & 0x03FFFFFF;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%spoll(%d) iTypCode=0x%X drvlocal.iOffset=%u statusReasonAux=%08x\n",
              modNamEMC, axisNo_, drvlocal.iTypCode, drvlocal.iOffset,
              statusReasonAux);
  } else if (drvlocal.iTypCode == 0x1E04) {
    readAuxBitNamesEnums();
    struct {
      uint8_t   currentValue[2];
      uint8_t   targetValue[2];
      uint8_t   statReasAux[4];
    } readback;
#if 0
    memset(&readback, 0, sizeof(readback));
#endif
    if (cached) {
      status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                                 &readback,
                                                 sizeof(readback));
    } else {
      status = pC_->getPlcMemoryViaADS(drvlocal.iOffset,
                                       &readback,
                                       sizeof(readback));
    }
#if 0
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%spoll(%d) curr=0x%02X 0x%02X tgt=0x%02X 0x%02X  statReasAux=0x%02X 0x%02X 0x%02X 0x%02X\n",
              modNamEMC, axisNo_,
              readback.currentValue[0],
              readback.currentValue[1],
              readback.targetValue[0],
              readback.targetValue[1],
              readback.statReasAux[0],
              readback.statReasAux[1],
              readback.statReasAux[2],
              readback.statReasAux[3]);
#endif
    if (status) {
      return status;
    }
    errorID = 0;
    homed = 1;
    setIntegerParamLog(pC_->motorStatusHomed_, homed, "homed");
    actPosition = (double)NETTOUINT(readback.currentValue);
    setIntegerParam(pC_->defAsynPara.pilsLonginActual_,
                    NETTOUINT(readback.currentValue));
    pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.pilsLonginActual_,
                                       asynSuccess);
    setIntegerParam(pC_->defAsynPara.pilsLonginTarget_,
                    NETTOUINT(readback.targetValue));
    pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.pilsLonginTarget_,
                                       asynSuccess);

    statusReasonAux = NETTOUINT(readback.statReasAux);
    setIntegerParam(pC_->defAsynPara.ethercatmcAuxBits07_, statusReasonAux & 0xFF);
    pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcAuxBits07_,
                                       asynSuccess);
    idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
    idxReasonBits = (statusReasonAux >> 24) & 0x0F;
    idxAuxBits    =  statusReasonAux  & 0x03FFFFFF;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%spoll(%d) iTypCode=0x%X drvlocal.iOffset=%u statusReasonAux=%08x actPosition=%f\n",
              modNamEMC, axisNo_, drvlocal.iTypCode, drvlocal.iOffset,
              statusReasonAux, actPosition);
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) iTypCode=0x%X\n",
              modNamEMC, axisNo_, drvlocal.iTypCode);
    return asynError;
  }
  drvlocal.hasProblem = 0;
  setIntegerParam(pC_->defAsynPara.ethercatmcStatusCode_, idxStatusCode);
  if ((idxStatusCode != drvlocal.dirty.idxStatusCode) ||
      (errorID != drvlocal.dirty.old_ErrorId)) {
    if (errorID) {
      asynPrint(pC_->pasynUserController_, traceMask,
                "%spoll(%d) statusReasonAux=0x%08X (%s) errorID=0x%04X \"%s\" actPos=%f\n",
                modNamEMC, axisNo_,
                statusReasonAux,
                idxStatusCodeTypeToStr(idxStatusCode),
                errorID, errStringFromErrId(errorID),
                actPosition);
    } else {
      asynPrint(pC_->pasynUserController_, traceMask,
                "%spoll(%d) statusReasonAux=0x%08X (%s) actPos=%f\n",
                modNamEMC, axisNo_,
                statusReasonAux,
                idxStatusCodeTypeToStr(idxStatusCode),
                actPosition);
    }
  }
  if (idxAuxBits != drvlocal.old_idxAuxBits) {
    /* This is for debugging only: The IOC log will show changed bits */
    pC_->changedAuxBits_to_ASCII(axisNo_,
                                 pC_->defAsynPara.ethercatmcNamAux0_,
                                 idxAuxBits, drvlocal.old_idxAuxBits);
    asynPrint(pC_->pasynUserController_, traceMask,
              "%spoll(%d) auxBitsOld=0x%06X new=0x%06X (%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s) actPos=%f\n",
              modNamEMC, axisNo_, drvlocal.old_idxAuxBits, idxAuxBits,
              pC_->ctrlLocal.changedAuxBits[0],  pC_->ctrlLocal.changedAuxBits[1],
              pC_->ctrlLocal.changedAuxBits[2],  pC_->ctrlLocal.changedAuxBits[3],
              pC_->ctrlLocal.changedAuxBits[4],  pC_->ctrlLocal.changedAuxBits[5],
              pC_->ctrlLocal.changedAuxBits[6],  pC_->ctrlLocal.changedAuxBits[7],
              pC_->ctrlLocal.changedAuxBits[8],  pC_->ctrlLocal.changedAuxBits[9],
              pC_->ctrlLocal.changedAuxBits[10], pC_->ctrlLocal.changedAuxBits[11],
              pC_->ctrlLocal.changedAuxBits[12], pC_->ctrlLocal.changedAuxBits[13],
              pC_->ctrlLocal.changedAuxBits[14], pC_->ctrlLocal.changedAuxBits[15],
              pC_->ctrlLocal.changedAuxBits[16], pC_->ctrlLocal.changedAuxBits[17],
              pC_->ctrlLocal.changedAuxBits[18], pC_->ctrlLocal.changedAuxBits[19],
              pC_->ctrlLocal.changedAuxBits[20], pC_->ctrlLocal.changedAuxBits[21],
              pC_->ctrlLocal.changedAuxBits[22], pC_->ctrlLocal.changedAuxBits[23],
              pC_->ctrlLocal.changedAuxBits[24], pC_->ctrlLocal.changedAuxBits[25],
              actPosition);
  }
  /* This is for EPICS records: after a re-connection,
     all bits should be written once */
  if (idxAuxBits != drvlocal.old_idxAuxBits ||
      idxAuxBits != drvlocal.dirty.old_idxAuxBits) {
    for (unsigned auxBitIdx = 0; auxBitIdx < MAX_AUX_BIT_AS_BI_RECORD; auxBitIdx++) {
      int function = drvlocal.asynFunctionAuxBitAsBiRecord[auxBitIdx];
      if (function) {
        int value = (idxAuxBits >> auxBitIdx) & 1;
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
                  "%spoll(%d) auxBitIdx=%u function=%d value=%d\n",
                  modNamEMC, axisNo_, auxBitIdx, function, value);
        setIntegerParam(function, value);
        if (drvlocal.old_idxAuxBits != drvlocal.dirty.old_idxAuxBits) {
          /* In the first cycle:
             old_idxAuxBits == 0, dirty.old_idxAuxBits == 0xFFFFFFFF
             Unset the alarm state for all aux bits that we have found
             after a connect or re-connect
          */
          pC_->setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
        }
      }
    }
  }
  switch (idxStatusCode) {
    /* After RESET, START, STOP the bits are not valid */
  case idxStatusCodeIDLE:
  case idxStatusCodeWARN:
    powerIsOn = 1;
    statusValid = 1;
    break;
  case idxStatusCodePOWEROFF:
    statusValid = 1;
    drvlocal.hasProblem = 1;
    powerIsOn = 0;
    break;
  case idxStatusCodeBUSY:
    powerIsOn = 1;
    statusValid = 1;
    nowMoving = true;
    break;
  case idxStatusCodeERROR:
    hasError = 1;
    statusValid = 1;
    drvlocal.hasProblem = 1;
    break;
  case idxStatusCodeRESET:
    positionValid = 0;
    setIntegerParam(pC_->motorStatusMoving_, 0);
    setIntegerParam(pC_->motorStatusDone_, 1);
    break;
  case idxStatusCodeSTART:
  case idxStatusCodeSTOP:
    nowMoving = true;
    break;
  default:
    drvlocal.hasProblem = 1;
  }
  *moving = nowMoving;
  if (positionValid) {
    double oldPositionValue;
    asynStatus oldPositionStatus;
    oldPositionStatus = pC_->getDoubleParam(axisNo_,
                                            pC_->motorPosition_,
                                            &oldPositionValue);
    if (oldPositionStatus == asynSuccess) {
      /* Use previous fActPosition and
         current fActPosition to calculate direction.*/
      if (actPosition > oldPositionValue) {
        setIntegerParam(pC_->motorStatusDirection_, 1);
      } else if (actPosition < oldPositionValue) {
        setIntegerParam(pC_->motorStatusDirection_, 0);
      }
    }
    setDoubleParam(pC_->motorPosition_, actPosition);
    setDoubleParam(pC_->motorEncoderPosition_, actPosition);
  }
  if (statusValid || nowMoving) {
    /* These 2 bits are important to inform the motorRecord
       when a motion is completed */
    setIntegerParam(pC_->motorStatusMoving_, nowMoving);
    setIntegerParam(pC_->motorStatusDone_, !nowMoving);
  }
  if (statusValid) {
    int hls = idxReasonBits & 0x8 ? 1 : 0;
    int lls = idxReasonBits & 0x4 ? 1 : 0;
    if (drvlocal.auxBitsLocalModeMask) {
      localMode = idxAuxBits & drvlocal.auxBitsLocalModeMask ? 1 : 0;;
      //nowMoving |= localMode;
    }
    setIntegerParamLog(pC_->motorStatusLowLimit_, lls,  "LLS");
    setIntegerParamLog(pC_->motorStatusHighLimit_, hls, "HLS");
    pC_->setUIntDigitalParam(axisNo_, pC_->defAsynPara.ethercatmcStatusBits_,
                             (epicsUInt32)statusReasonAux,
                             0x0FFFFFFF, 0x0FFFFFFF);
    setIntegerParam(pC_->defAsynPara.ethercatmcMcuErr_, hasError);
    if (drvlocal.auxBitsNotHomedMask) {
      homed = idxAuxBits & drvlocal.auxBitsNotHomedMask ? 0 : 1;
      setIntegerParamLog(pC_->motorStatusHomed_, homed, "homed");
#ifdef motorNotHomedProblemString
      if (!homed) {
        drvlocal.hasProblem = 1;
      }
#endif
    }
    if (drvlocal.auxBitsHomeSwitchMask) {
      int homeSwitch = idxAuxBits & drvlocal.auxBitsHomeSwitchMask ? 0 : 1;
      setIntegerParamLog(pC_->motorStatusAtHome_, homeSwitch, "atHome");
    }
    if (drvlocal.auxBitsEnabledMask) {
      powerIsOn = idxAuxBits & drvlocal.auxBitsEnabledMask ? 1 : 0;
    }
    if (hasError || errorID) {
      char sErrorMessage[40];
      const char *errIdString = errStringFromErrId(errorID);
      const char charEorW = hasError ? 'E' : 'W';
      memset(&sErrorMessage[0], 0, sizeof(sErrorMessage));
      if (!powerIsOn) {
        /*
         * It is important to know, if the motor can be disconnected
         * on e.g. a sample stage.
         * Let the generic driver write PowerOff and hide the error text so long
         * The error LED is still there
         */
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                 "%c: (PowerOff) %X", charEorW, errorID);
      } else if (errIdString[0]) {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1, "%c: %s %X",
                 charEorW, errIdString, errorID);
      } else {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                 "%c: TwinCAT Err %X", charEorW, errorID);
      }
      msgTxtFromDriver = sErrorMessage;
    } else if (localMode) {
      msgTxtFromDriver = "localMode";
      hasError = -1;
    }
    /* Update if we have an error now.
       Update even if we had an error before - it may have gone now,
       and the we need to set the NULL pointer */
    if (hasError || errorID ||
        drvlocal.dirty.old_hasError || drvlocal.dirty.old_ErrorId) {
      updateMsgTxtFromDriver(msgTxtFromDriver);
    }
    drvlocal.dirty.old_hasError = hasError;
    setIntegerParam(pC_->defAsynPara.ethercatmcStatusCode_, idxStatusCode);
    setIntegerParam(pC_->motorStatusProblem_, drvlocal.hasProblem | localMode);
    setIntegerParamLog(pC_->motorStatusPowerOn_, powerIsOn, "powerOn");
  }

  if (positionValid) {
    double cfgPmax, cfgPmin;
    int pilsLonginTargetValue;
    asynStatus pilsLonginTargetStatus;

    pilsLonginTargetStatus = pC_->getIntegerParam(axisNo_,
                                                  pC_->defAsynPara.pilsLonginTarget_,
                                                  &pilsLonginTargetValue);

    if (!(pC_->getDoubleParam(axisNo_,
                              pC_->defAsynPara.ethercatmcCfgPMAX_RB_, &cfgPmax))) {
      /* readback */
      if (pilsLonginTargetStatus == asynSuccess &&
          pilsLonginTargetValue == (int)cfgPmax) {
        /* output record with readback: Use base class */
        int function = pC_->defAsynPara.pilsBoMinMax_;
        asynMotorAxis::setIntegerParam(function, 1);
        pC_->setAlarmStatusSeverityWrapper(axisNo_, function, asynSuccess);
      }
    }
    if (!(pC_->getDoubleParam(axisNo_,
                              pC_->defAsynPara.ethercatmcCfgPMIN_RB_, &cfgPmin))) {
      /* readback */
      if (pilsLonginTargetStatus == asynSuccess &&
          pilsLonginTargetValue == (int)cfgPmin) {
        /* output record with readback: Use base class */
        int function = pC_->defAsynPara.pilsBoMinMax_;
        asynMotorAxis::setIntegerParam(function, 0);
        pC_->setAlarmStatusSeverityWrapper(axisNo_, function,  asynSuccess);
      }
    }
  }

  if (pC_->ctrlLocal.systemUTCtimeOffset)  {
    /* Motor position in "user coordinates" with UTC time from PTP */
    double motorRecOffset = 0.0;
    int motorRecDirection = 1;
    int ethercatmcPTPallGood = 0;
    asynStatus RBV_TSEstatus = asynDisabled;
    double ethercatmcRBV_TSE = 0.0;

    pC_->getIntegerParam(0, pC_->defAsynPara.ethercatmcPTPallGood_, &ethercatmcPTPallGood);
    if ((!pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_,
                              &motorRecOffset)) &&
        (!pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,
                               &motorRecDirection)) &&
        (positionValid && statusValid && homed && (ethercatmcPTPallGood == 1))) {
      RBV_TSEstatus = asynSuccess;
    }
    /* direction == 1 means "negative" */
    motorRecDirection = motorRecDirection ? -1 : 1;
    ethercatmcRBV_TSE = actPosition *  motorRecDirection + motorRecOffset;
    setDoubleParam(pC_->defAsynPara.ethercatmcRBV_TSE_, ethercatmcRBV_TSE);
    pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->defAsynPara.ethercatmcRBV_TSE_,
                                       RBV_TSEstatus);
  }

  if ((paramCtrl & PARAM_IF_CMD_MASK) == PARAM_IF_CMD_DONE) {
    unsigned paramIndex = paramCtrl & PARAM_IF_IDX_MASK;
    drvlocal.param_read_ok_once[paramIndex] = 1;
      asynPrint(pC_->pasynUserController_,
              ASYN_TRACE_FLOW,
              "%spoll(%d) paramCtrl=%s (0x%x) paramValue=%f\n",
              modNamEMC, axisNo_,
              plcParamIndexTxtFromParamIndex(paramIndex),
              paramCtrl, paramfValue);
    if ((paramCtrl != drvlocal.old_paramCtrl) ||
        (paramfValue != drvlocal.old_paramValue)) {
      /* The enums must have been read.
         Only read real parameters, not functions */
      if (!drvlocal.enumparam_read_id[paramIndex] &&
          paramIndexIsParameterToPoll(paramIndex)) {
        int initial = 0;
        pC_->parameterFloatReadBack(axisNo_,
                                    initial,
                                    paramIndex,
                                    paramfValue);
      }
      drvlocal.old_paramCtrl = paramCtrl;
      drvlocal.old_paramValue = paramfValue;
    }
  }

  /* Read back the parameters one by one */
  if (drvlocal.pollScaling && drvlocal.paramIfOffset &&
      (paramCtrl & PARAM_IF_ACK_MASK)) {
    /* Increment */
    drvlocal.pollNowIdx++;
    int counter = 255;
    uint16_t paramIndex = drvlocal.pollNowParams[drvlocal.pollNowIdx];
    while (paramIndex && paramIndexIsReadLaterInBackground(paramIndex) && (counter > 0)) {
      switch (drvlocal.pollScaling) {
      case PollScalingOnce:
        {
          if (drvlocal.param_read_ok_once[paramIndex]) {
            drvlocal.pollNowIdx++;
            paramIndex = drvlocal.pollNowParams[drvlocal.pollNowIdx];
            if (!paramIndex) {
              /* wrap around */
              drvlocal.pollNowIdx = 0;
              paramIndex = drvlocal.pollNowParams[drvlocal.pollNowIdx];
            }
          } else {
            counter = 0;
          }
        }
        break;
      default:
      case PollScalingNo:
      case PollScalingCyclic:
        /* fall through */
        counter = 0;
        break;
      }
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
                "%spollParam(%d)pollNowIdx=%u paramIndex=%u pollInBG=%d pollScaling=%d param_read_ok_once=%d counter=%d\n",
                modNamEMC, axisNo_,
                drvlocal.pollNowIdx,
                paramIndex, paramIndexIsReadLaterInBackground(paramIndex),
                drvlocal.pollScaling,
                drvlocal.param_read_ok_once[paramIndex],
                counter);
      counter--;
    }
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%spollParam(%d)pollNowIdx=%u\n",
              modNamEMC, axisNo_, drvlocal.pollNowIdx);

    if (!drvlocal.pollNowParams[drvlocal.pollNowIdx]) {
      /* The list is 0 terminated */
      drvlocal.pollNowIdx = 0;
      /* Take the chance to read the enums
         In theory, this can be done earlier - but
         the record may not have registered the callback yet
      */
      if (!drvlocal.hasPolledAllEnums) {
        unsigned paramIndex;
        drvlocal.hasPolledAllEnums = 1; /* May be overwritten below */
        for (paramIndex = 0; paramIndex < (sizeof(drvlocal.PILSparamPerm) /
                                           sizeof(drvlocal.PILSparamPerm[0]));
             paramIndex++) {
          if (drvlocal.enumparam_read_id[paramIndex]) {
            unsigned enumparam_read_id = drvlocal.enumparam_read_id[paramIndex];
            status = pC_->indexerV3readParameterEnums(this,
                                                      paramIndex,
                                                      enumparam_read_id,
                                                      drvlocal.lenInPlcPara);
            /* We must read the enums before reading the value
               If reading ths enum fails, do not read the value */
            if (!status) {
              status = pC_->indexerParamRead(this,
                                             drvlocal.paramIfOffset,
                                             paramIndex,
                                             &paramfValue);
              if (!status) {
                int initial = 1;
                pC_->parameterFloatReadBack(axisNo_,
                                            initial,
                                            paramIndex,
                                            paramfValue);
              }
            }
            if (status) {
              /* reading failed, try again later */
              drvlocal.hasPolledAllEnums = 0;
            } else {
              /* Once read succesfully , do not read again */
              drvlocal.enumparam_read_id[paramIndex] = 0;
            }
          }
        }
      }
    }
    if (drvlocal.pollNowParams[drvlocal.pollNowIdx]) {
      uint16_t paramIndex = drvlocal.pollNowParams[drvlocal.pollNowIdx];
      uint16_t newParamCtrl = PARAM_IF_CMD_DOREAD + paramIndex;
      asynPrint(pC_->pasynUserController_,
                ASYN_TRACE_FLOW,
                "%spollNext(%d) paramCtrl=%s (0x%x) paramValue=%f\n",
                modNamEMC, axisNo_,
                plcParamIndexTxtFromParamIndex(paramIndex),
                paramCtrl, paramfValue);
      pC_->setPlcMemoryInteger(drvlocal.paramIfOffset,
                               newParamCtrl, sizeof(newParamCtrl));
    }
  }
  drvlocal.old_idxAuxBits        = idxAuxBits;
  drvlocal.dirty.old_idxAuxBits  = idxAuxBits;
  drvlocal.dirty.idxStatusCode   = idxStatusCode;
  drvlocal.dirty.old_ErrorId = errorID;
  callParamCallbacks();
  return status;
}

void ethercatmcIndexerAxis::readAuxBitNamesEnums(void)
{

  /* Our internal maximumum for AUX bits resulting in an mbbi */
#define MAX_AUX_BIT_FOR_ENUM 8

  /* asyn/asyn/devEpics/devAsynInt32.c */
#define MAX_ENUM_STRING_SIZE 26

  struct {
    char enumChars[MAX_AUX_BIT_FOR_ENUM][MAX_ENUM_STRING_SIZE];
    char *enumStrings[MAX_AUX_BIT_FOR_ENUM];
    int enumValues[MAX_AUX_BIT_FOR_ENUM];
    int enumSeverities[MAX_AUX_BIT_FOR_ENUM];
  } auxBitEnumsForAsyn;
  unsigned auxBitIdx;
  unsigned numsAuxBitsForEnum = 0;

  memset (&auxBitEnumsForAsyn, 0, sizeof(auxBitEnumsForAsyn));

  for (auxBitIdx = 0; auxBitIdx < MAX_AUX_BIT_FOR_ENUM; auxBitIdx++) {
    int function = pC_->defAsynPara.ethercatmcNamAux0_ + (int)auxBitIdx;
    asynStatus status;
    /* Leave one byte for '\0' */
    int length = (int)sizeof(auxBitEnumsForAsyn.enumChars[auxBitIdx]) - 1;
    auxBitEnumsForAsyn.enumStrings[auxBitIdx] = &auxBitEnumsForAsyn.enumChars[auxBitIdx][0];
    status = pC_->getStringParam(axisNo_, function, length,
                                 auxBitEnumsForAsyn.enumStrings[auxBitIdx]);
    if (status) {
      break;
    }
    auxBitEnumsForAsyn.enumValues[auxBitIdx] = 1 << auxBitIdx;
    numsAuxBitsForEnum++;
  }
  pC_->doCallbacksEnum(auxBitEnumsForAsyn.enumStrings,
                       auxBitEnumsForAsyn.enumValues,
                       auxBitEnumsForAsyn.enumSeverities,
                       numsAuxBitsForEnum,
                       pC_->defAsynPara.ethercatmcAuxBits07_, axisNo_);
}

asynStatus ethercatmcIndexerAxis::resetAxis(void)
{
  return writeCmdRegisster(idxStatusCodeRESET);
}
bool ethercatmcIndexerAxis::pollPowerIsOn(void)
{
  bool cached = false;
  bool moving;
  asynStatus status = doThePoll(cached, &moving);
  if (status) {
    return false;
  }
  int powerIsOn = 0;
  pC_->getIntegerParam(axisNo_, pC_->motorStatusPowerOn_, &powerIsOn);
  return powerIsOn ? true : false;
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus ethercatmcIndexerAxis::setClosedLoop(bool closedLoop)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_,
            (int)closedLoop);
  double fValue = closedLoop ? 0.0 : 1.0; /* 1.0 means disable */
  asynStatus status =  pC_->indexerParamWrite(this, PARAM_IDX_OPMODE_AUTO_UINT,
                                              fValue, NULL);
  //The PILS interface in the MCU should be able to return the actual
  // value, if we change NULL from above with an &double,
  // which would save us one transaction
  // Need to check the PLC code first, before changing things here
  (void)pollPowerIsOn();
  return status;
}

asynStatus ethercatmcIndexerAxis::setGenericIntegerParam(int function, int value)
{
  pilsAsynDevInfo_type *pPilsAsynDevInfo;
  pPilsAsynDevInfo = pC_->findIndexerOutputDevice(axisNo_, function,
                                                  asynParamInt32);
  if (pPilsAsynDevInfo) {
    const char *paramName = NULL;
    if (pC_->getParamName(axisNo_, function, &paramName)) paramName = "";
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetGenericIntegerParam(%d %s offset=%u)=%d\n",
              modNamEMC, axisNo_,
              paramName,
              pPilsAsynDevInfo->outputOffset, value);
    return pC_->setPlcMemoryInteger(pPilsAsynDevInfo->outputOffset,
                                    value,
                                    pPilsAsynDevInfo->lenInPLC);
  }
  return asynSuccess;
}


asynStatus ethercatmcIndexerAxis::setIntegerParam(int function, int value)
{
  asynStatus status = asynSuccess;
  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorUpdateStatus_)=%d\n",
              modNamEMC, axisNo_, value);

  } else if (function == pC_->motorStatusCommsError_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%ssetIntegerParam(%d pC_->motorStatusCommsError_)=%d\n",
              modNamEMC, axisNo_, value);
    if (value  && !drvlocal.dirty.initialPollNeeded ) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                "%s Communication error(%d)\n", modNamEMC, axisNo_);
      for (unsigned auxBitIdx = 0; auxBitIdx < MAX_AUX_BIT_AS_BI_RECORD; auxBitIdx++) {
        int function = drvlocal.asynFunctionAuxBitAsBiRecord[auxBitIdx];
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                  "%smotorStatusCommsError_(%d) auxBitIdx=%u function=%d\n",
                  modNamEMC, axisNo_, auxBitIdx, function);
        if (function) {
          pC_->setAlarmStatusSeverityWrapper(axisNo_, function, asynDisconnected);
        }
      }
      memset(&drvlocal, 0, sizeof(drvlocal));
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
    }
#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->defAsynPara.ethercatmcErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
  } else if (function == pC_->defAsynPara.ethercatmcCfgDHLM_En_) {
    unsigned paramIndex = PARAM_IDX_USR_MAX_EN_UINT;
    double valueRB = -1;
    if (drvlocal.PILSparamPerm[paramIndex] != PILSparamPermWrite) {
      paramIndex = PARAM_IDX_USR_MAX_EN_FLOAT;
    }
    status = pC_->indexerParamWrite(this, paramIndex, value, &valueRB);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d defAsynPara.ethercatmcCfgDHLM_En)=%d  status=%s(%d)\n",
              modNamEMC, axisNo_, value,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
    }
  } else if (function == pC_->defAsynPara.ethercatmcCfgDLLM_En_) {
    unsigned paramIndex = PARAM_IDX_USR_MIN_EN_UINT;
    double valueRB = -1;
    if (drvlocal.PILSparamPerm[paramIndex] != PILSparamPermWrite) {
      paramIndex = PARAM_IDX_USR_MIN_EN_FLOAT;
    }
    status = pC_->indexerParamWrite(this, paramIndex, value, &valueRB);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d defAsynPara.ethercatmcCfgDLLM_En)=%d status=%s(%d)\n",
              modNamEMC, axisNo_, value,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
    }
  } else if (function == pC_->defAsynPara.pilsBoMinMax_) {
    int function = pC_->defAsynPara.ethercatmcCfgPMIN_RB_;
    if (value == 1) {
      function = pC_->defAsynPara.ethercatmcCfgPMAX_RB_;
    }
    double position;
    status = pC_->getDoubleParam(axisNo_, function, &position);
    if (status == asynSuccess) {
      switch (drvlocal.iTypCode) {
      case 0x1E04:
      case 0x5008:
      case 0x5010:
        {
          /* Devices that need a "start" command, like motors */
          int relative = 0;
          double minVelocity = -1.0; /* No setting/changing of velo/accs */
          double maxVelocity = -1.0;
          double acceleration = -1.0;
          /* Use move() to set the position and to send a start */
          status = move(position, relative, minVelocity, maxVelocity, acceleration);
        }
        break;
      default:
        status = setGenericIntegerParam(function, value);
      }
    } else {
      status = asynError;
    }
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d pilsBoMinMax_)=%d iTypCode=0x%04x status=%s(%d)\n",
              modNamEMC, axisNo_, value, drvlocal.iTypCode,
              ethercatmcstrStatus(status), (int)status);
  } else if (function == pC_->defAsynPara.pilsLongoutRecord_) {
    switch (drvlocal.iTypCode) {
    case 0x1E04:
    case 0x5008:
    case 0x5010:
      {
        /* Devices that need a "start" command, like motors */
          double position = (double)value;
          int relative = 0;
          double minVelocity = -1.0; /* No setting/changing of velo/accs */
          double maxVelocity = -1.0;
          double acceleration = -1.0;
          /* Use move() to set the position and to send a start */
          status = move(position, relative, minVelocity, maxVelocity, acceleration);
        }
      break;
    default:
      status = setGenericIntegerParam(function, value);
    }
  } else if (function == pC_->defAsynPara.ethercatmcPollScaling_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d PollScaling_)=%d\n",
              modNamEMC, axisNo_, value);
    drvlocal.pollScaling = value;
  } else {
    status = setGenericIntegerParam(function, value);
  }
  if (function == pC_->defAsynPara.pilsLonginActual_ || function == pC_->defAsynPara.pilsLonginTarget_) {
  }
  // Update the parameter in the base class
  (void)asynMotorAxis::setIntegerParam(function, value);
  return status;
}

asynStatus ethercatmcIndexerAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  if (function == pC_->defAsynPara.ethercatmcCfgDHLM_) {
    static const unsigned paramIndex = PARAM_IDX_USR_MAX_FLOAT;
    double valueRB = -1;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d defAsynPara.ethercatmcCfgDHLM_)=%g\n", modNamEMC, axisNo_, value);
    status = pC_->indexerParamWrite(this, paramIndex, value, &valueRB);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else if (function == pC_->defAsynPara.ethercatmcCfgDLLM_) {
    double valueRB = -1;
    static const unsigned paramIndex = PARAM_IDX_USR_MIN_FLOAT;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d defAsynPara.ethercatmcCfgDLLM_)=%g\n", modNamEMC, axisNo_, value);
    status = pC_->indexerParamWrite(this, paramIndex, value, &valueRB);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOnDelay_)=%g\n", modNamEMC, axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d motorPowerOffDelay_=%g\n", modNamEMC, axisNo_, value);
#endif
  }

  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;

}

asynStatus ethercatmcIndexerAxis::setStringParamDbgStrToMcu(const char *value)
{
  asynStatus status;
  const char * const Main_this_str = "Main.this.";
  const char * const Sim_this_str = "Sim.this.";
  const char * const Sim_this_ads_str = "Sim.this.ads.";
  /* The special device structure. */
  struct {
    /* 2 bytes control, 46 payload */
    uint8_t   busyLen[2];
    char      value[46];
  } netDevice0518interface;
  int valueLen;


  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetStringParamDbgStrToMcu(%d)=\"%s\"\n",
            modNamEMC, axisNo_, value);
  /* empty strings are not send to the controller */
  if (!value[0]) return asynSuccess;
  /* No special device found */
  if (!pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset) return asynError;



  if (!strncmp(value, Sim_this_ads_str, strlen(Sim_this_ads_str))) {
    /* caput IOC:m1-DbgStrToMCU Sim.this.ads.simulatedNetworkProblem=1 */
    valueLen = snprintf(netDevice0518interface.value,
                        sizeof(netDevice0518interface.value),
                        "%s;\n", value);
  } else if (!strncmp(value, Main_this_str, strlen(Main_this_str))) {
    /* Check the string. E.g. Main.this. and Sim.this. are passed
       as Main.M1 or Sim.M1 */
    valueLen = snprintf(netDevice0518interface.value,
                        sizeof(netDevice0518interface.value),
                        "Main.M%d.%s;\n",
                        axisNo_, value + strlen(Main_this_str));
  } else if (!strncmp(value, Sim_this_str, strlen(Sim_this_str))) {
    /* caput IOC:m1-DbgStrToMCU Sim.this.log=M1.log */
    valueLen = snprintf(netDevice0518interface.value,
                        sizeof(netDevice0518interface.value),
                        "Sim.M%d.%s;\n",
                        axisNo_, value + strlen(Sim_this_str));
  } else {
    /* If we come here, the command was not understood */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
              "%ssetStringParamDbgStrToMcu (%d) illegal value=\"%s\"\n"
              "allowed: \"%s\" or \"%s\"\n",
              "ethercatmcIndexerAxis", axisNo_, value,
              Main_this_str,
              Sim_this_str);
    return asynError;
  }
  if (valueLen < 0 || (unsigned)valueLen >= sizeof(netDevice0518interface.value)) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
              "%ssetStringParamDbgStrToMcu (%d) valueLen=%d value=\"%s\"\n",
              "ethercatmcIndexerAxis", axisNo_,
              valueLen, value);
    return asynError;
  }
  UINTTONET(valueLen, netDevice0518interface.busyLen);

  /* obey the handshake */
  status = pC_->indexerWaitSpecialDeviceIdle(pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset);
  if (status) return status;

  status = pC_->setPlcMemoryViaADS(pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset,
                                 &netDevice0518interface,
                                 pC_->ctrlLocal.specialDbgStrToMcuDeviceLength);
  if (status) return status;
  /* Wait for the MCU to acknowledge the command */
  status = pC_->indexerWaitSpecialDeviceIdle(pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset);
  if (status) return status;
  status = pC_->getPlcMemoryViaADS(pC_->ctrlLocal.specialDbgStrToMcuDeviceOffset,
                                   &netDevice0518interface,
                                   pC_->ctrlLocal.specialDbgStrToMcuDeviceLength);
  if (status) return status;
  if (strcmp(netDevice0518interface.value, "OK")) status = asynError;

  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetStringParamDbgStrToMcu(%d)=\"%s\" ret=\"%s\" status=%d\n",
            modNamEMC, axisNo_, value,
            netDevice0518interface.value,
            (int)status);
  return status;
}

asynStatus ethercatmcIndexerAxis::setStringParam(int function, const char *value)
{
  if (function == pC_->defAsynPara.ethercatmcDbgStrToLog_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetStringParamDbgStrToLog(%d)=\"%s\"\n",
              modNamEMC, axisNo_, value);
  } else if (function == pC_->defAsynPara.ethercatmcDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  }
  /* Call base class method */
  return asynMotorAxis::setStringParam(function, value);
}

#ifndef motorMessageTextString
void ethercatmcIndexerAxis::updateMsgTxtFromDriver(const char *value)
{
  (void)value;
}
#endif
