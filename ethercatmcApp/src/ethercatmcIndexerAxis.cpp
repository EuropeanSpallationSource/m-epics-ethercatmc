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


typedef enum {
  idxStatusCodeRESET    = 0,
  idxStatusCodeIDLE     = 1,
  idxStatusCodePOWEROFF = 2,
  idxStatusCodeWARN     = 3,
  idxStatusCodeERR4     = 4,
  idxStatusCodeSTART    = 5,
  idxStatusCodeBUSY     = 6,
  idxStatusCodeSTOP     = 7,
  idxStatusCodeERROR    = 8,
  idxStatusCodeERR9     = 9,
  idxStatusCodeERR10    = 10,
  idxStatusCodeERR11    = 11,
  idxStatusCodeERR12    = 12,
  idxStatusCodeERR13    = 13,
  idxStatusCodeERR14    = 14,
  idxStatusCodeERR15    = 15
} idxStatusCodeType;

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
#ifdef  motorNotHomedProblemString
  setIntegerParam(pC_->motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
  setStringParam(pC_->ethercatmcNamBit25_, "Dynamic problem, timeout");
  setStringParam(pC_->ethercatmcNamBit24_, "Static problem, inhibit");

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
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d) iTypCode=0x%X\n",
              modNamEMC, axisNo_, drvlocal.iTypCode);
  }
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

  if (maxVelocity > 0.0) {
    double oldValue, valueRB;
    status = pC_->getDoubleParam(axisNo_, pC_->ethercatmcVel_RB_, &oldValue);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%smove (%d) ethercatmcVel status=%s (%d)\n",
              "ethercatmcIndexerAxis", axisNo_,
              ethercatmcstrStatus(status), (int)status);
    if ((status != asynParamUndefined) && (maxVelocity != oldValue)) {
      status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                      PARAM_IDX_SPEED_FLOAT,
                                      drvlocal.lenInPlcPara,
                                      maxVelocity, &valueRB);
      if (status) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                  "%smove (%d) status=%s (%d)\n",
                  "ethercatmcIndexerAxis", axisNo_,
                  ethercatmcstrStatus(status), (int)status);
        return status;
      }
      setDoubleParam(pC_->ethercatmcVel_RB_, maxVelocity);
    }
  }
  if (acceleration > 0.0) {
    double oldValue, valueRB;
    status = pC_->getDoubleParam(axisNo_, pC_->ethercatmcAcc_RB_, &oldValue);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_FLOW,
              "%smove (%d) ethercatmcAcc status=%s (%d)\n",
              "ethercatmcIndexerAxis", axisNo_,
              ethercatmcstrStatus(status), (int)status);
    if ((status != asynParamUndefined) && (acceleration != oldValue)) {
      status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                      PARAM_IDX_ACCEL_FLOAT,
                                      drvlocal.lenInPlcPara,
                                      acceleration, &valueRB);
      if (status) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                  "%smove (%d) status=%s (%d)\n",
                  "ethercatmcIndexerAxis", axisNo_,
                  ethercatmcstrStatus(status), (int)status);
        return status;
      }
      setDoubleParam(pC_->ethercatmcAcc_RB_, acceleration);
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
  status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                  PARAM_IDX_FUN_REFERENCE,
                                  drvlocal.lenInPlcPara,
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
  if (acceleration > 0.0) {
    double oldValue, valueRB;
    pC_->getDoubleParam(axisNo_, pC_->ethercatmcAcc_RB_, &oldValue);
    if (acceleration != oldValue) {
      status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                      PARAM_IDX_ACCEL_FLOAT,
                                      drvlocal.lenInPlcPara,
                                      acceleration, &valueRB);
      if (status) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR,
                  "%smoveVelocity (%d) status=%s (%d)\n",
                  "ethercatmcIndexerAxis", axisNo_,
                  ethercatmcstrStatus(status), (int)status);
        return status;
      }
      setDoubleParam(pC_->ethercatmcAcc_RB_, acceleration);
    }
  }
  status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                  PARAM_IDX_FUN_MOVE_VELOCITY,
                                  drvlocal.lenInPlcPara,
                                  maxVelocity, &veloRB);
  asynPrint(pC_->pasynUserController_, traceMask,
            "%smoveVelocity (%d) veloRB=%f status=%s(%d)\n",
            "ethercatmcIndexerAxis", axisNo_,
            veloRB, ethercatmcstrStatus(status), (int)status);
  if (status) return status;
  setDoubleParam(pC_->ethercatmcVel_RB_, veloRB);
  return status;
}


/**
 * See asynMotorAxis::setPosition
 */
asynStatus ethercatmcIndexerAxis::setPosition(double value)
{
  asynStatus status = asynError;
  if (drvlocal.paramIfOffset) {
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    PARAM_IDX_FUN_SET_POSITION,
                                    drvlocal.lenInPlcPara,
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
  } else if (drvlocal.iTypCode == 0x5010) {
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

void ethercatmcIndexerAxis::setAxisID(unsigned axisID)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetAxisID(%u) %d\n",
            modNamEMC, axisNo_, axisID);
  drvlocal.axisID = axisID;
  setIntegerParamLog(pC_->ethercatmcCfgAxisID_RB_, drvlocal.axisID,"axisID");
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

/** Polls the axis.
 * This function reads the motor position, the limit status, the home status,
 * the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving
 * (true) or done (false). */
asynStatus ethercatmcIndexerAxis::poll(bool *moving)
{
  asynStatus status = asynSuccess;
  unsigned traceMask = ASYN_TRACE_INFO;
  const char *msgTxtFromDriver = NULL;
  //double targetPosition = 0.0;
  double actPosition = 0.0;
  double paramfValue = 0.0;
  unsigned paramiValue = 0;
  unsigned statusReasonAux, paramCtrl;
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
  int pollReadBackInBackGround = 1;
  int homed = 0;

  /* Don't leave *moving un-initialized, if we return early */
  *moving = false;
  if (!drvlocal.iTypCode) {
    /* No axis, (may be dummy-axis 0), return */
    return asynSuccess;
  }
  if (!drvlocal.iOffset) return asynSuccess;

  if (drvlocal.dirty.initialPollNeeded) {
    status = pC_->indexerReadAxisParameters(this, drvlocal.devNum,
                                            drvlocal.iOffset,
                                            drvlocal.lenInPlcPara);
    if (!status) {
      drvlocal.dirty.initialPollNeeded = 0;
      setIntegerParam(pC_->motorStatusCommsError_, 0);
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
    status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                               &readback,
                                               sizeof(readback));
    if (status) {
      return status;
    }
    actPosition = NETTODOUBLE(readback.actPos);
    //targetPosition = NETTODOUBLE(readback.targtPos);
    statusReasonAux16 = NETTOUINT(readback.statReasAux);
    paramCtrl = NETTOUINT(readback.paramCtrl);
    paramfValue = NETTODOUBLE(readback.paramValue);
    paramiValue = NETTOUINT(readback.paramValue);
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
    pC_->getIntegerParam(axisNo_, pC_->ethercatmcErrId_, &errorID);
  } else if (drvlocal.iTypCode == 0x5010) {
    struct {
      uint8_t   actPos[8];
      uint8_t   targtPos[8];
      uint8_t   statReasAux[4];
      uint8_t   errorID[2];
      uint8_t   paramCtrl[2];
      uint8_t   paramValue[8];
    } readback;
    status = pC_->getPlcMemoryFromProcessImage(drvlocal.iOffset,
                                               &readback,
                                               sizeof(readback));
    if (status) {
      return status;
    }
    actPosition = NETTODOUBLE(readback.actPos);
    //targetPosition = NETTODOUBLE(readback.targtPos);
    statusReasonAux = NETTOUINT(readback.statReasAux);
    paramCtrl = NETTOUINT(readback.paramCtrl);
    paramfValue = NETTODOUBLE(readback.paramValue);
    paramiValue = NETTOUINT(readback.paramValue);

    /* Specific for 5010 */
    errorID = (int)NETTOUINT(readback.errorID);
    setIntegerParam(pC_->ethercatmcErrId_, errorID);

    idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
    idxReasonBits = (statusReasonAux >> 24) & 0x0F;
    idxAuxBits    =  statusReasonAux  & 0x0FFFFFF;
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%spoll(%d) iTypCode=0x%X\n",
              modNamEMC, axisNo_, drvlocal.iTypCode);
    return asynError;
  }
  drvlocal.hasProblem = 0;
  setIntegerParam(pC_->ethercatmcStatusCode_, idxStatusCode);
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
    drvlocal.dirty.old_ErrorId = errorID;
  }
  if (idxAuxBits != drvlocal.old_idxAuxBits) {
    char changedNames[MAX_AUX_BIT_SHOWN][36];
    unsigned changed = idxAuxBits ^ drvlocal.old_idxAuxBits;
    unsigned auxBitIdx;
    memset(&changedNames, 0, sizeof(changedNames));
    for (auxBitIdx = 0; auxBitIdx < MAX_AUX_BIT_SHOWN; auxBitIdx++) {
      if ((changed >> auxBitIdx) & 0x01) {
        asynStatus status;
        int function = (int)(pC_->ethercatmcNamAux0_ + auxBitIdx);
        /* Leave the first character for '+' or '-',
           leave one byte for '\0' */
        int length = (int)sizeof(changedNames[auxBitIdx]) - 2;
        status = pC_->getStringParam(axisNo_,
                                     function,
                                     length,
                                     &changedNames[auxBitIdx][1]);
        if (status == asynSuccess) {
          /* the name of "aux bits without a name" is never written,
             so that we don't show it here */
          if ((idxAuxBits >> auxBitIdx) & 0x01) {
            changedNames[auxBitIdx][0] = '+';
          } else {
            changedNames[auxBitIdx][0] = '-';
          }
        }
      }
    }
    asynPrint(pC_->pasynUserController_, traceMask,
              "%spoll(%d) auxBitsOld=0x%04X new=0x%04X (%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s) actPos=%f\n",
              modNamEMC, axisNo_, drvlocal.old_idxAuxBits, idxAuxBits,
              changedNames[0],
              changedNames[1],
              changedNames[2],
              changedNames[3],
              changedNames[4],
              changedNames[5],
              changedNames[6],
              changedNames[7],
              changedNames[8],
              changedNames[9],
              changedNames[10],
              changedNames[11],
              changedNames[12],
              changedNames[13],
              changedNames[14],
              changedNames[15],
              changedNames[16],
              changedNames[17],
              changedNames[18],
              changedNames[19],
              changedNames[20],
              changedNames[21],
              changedNames[22],
              changedNames[23],
              actPosition);
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
  if (statusValid) {
    int hls = idxReasonBits & 0x8 ? 1 : 0;
    int lls = idxReasonBits & 0x4 ? 1 : 0;
    if (drvlocal.auxBitsLocalModeMask) {
      localMode = idxAuxBits & drvlocal.auxBitsLocalModeMask ? 1 : 0;;
      //nowMoving |= localMode;
    }
    setIntegerParamLog(pC_->motorStatusLowLimit_, lls,  "LLS");
    setIntegerParamLog(pC_->motorStatusHighLimit_, hls, "HLS");
    setIntegerParam(pC_->motorStatusMoving_, nowMoving);
    setIntegerParam(pC_->motorStatusDone_, !nowMoving);
    pC_->setUIntDigitalParam(axisNo_, pC_->ethercatmcStatusBits_,
                             (epicsUInt32)statusReasonAux,
                             0x0FFFFFF, 0x0FFFFFF);
    setIntegerParam(pC_->ethercatmcErr_, hasError);
    if (drvlocal.auxBitsNotHomedMask) {
      homed = idxAuxBits & drvlocal.auxBitsNotHomedMask ? 0 : 1;
      setIntegerParamLog(pC_->motorStatusHomed_, homed, "homed");
      if (!homed) {
        drvlocal.hasProblem = 1;
      }
    }
    if (drvlocal.auxBitsHomeSwitchMask) {
      int homeSwitch = idxAuxBits & drvlocal.auxBitsHomeSwitchMask ? 0 : 1;
      setIntegerParamLog(pC_->motorStatusAtHome_, homeSwitch, "atHome");
    }
    if (drvlocal.auxBitsEnabledMask) {
      powerIsOn = idxAuxBits & drvlocal.auxBitsEnabledMask ? 1 : 0;
    }
    if (!powerIsOn) {
      /*
       * It is more important to know, if the motor can be disconnected
       * on e.g. a sample stage.
       * Let the generic driver write PowerOff and hide the error text so long
       * The error LED is still there
       */
      hasError = 0;
    } else if (hasError) {
      char sErrorMessage[40];
      const char *errIdString = errStringFromErrId(errorID);
      memset(&sErrorMessage[0], 0, sizeof(sErrorMessage));
      if (errIdString[0]) {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1, "E: %s %X",
                 errIdString, errorID);
      } else {
        snprintf(sErrorMessage, sizeof(sErrorMessage)-1,
                 "E: TwinCAT Err %X", errorID);
      }
      msgTxtFromDriver = sErrorMessage;
    } else if (localMode) {
      msgTxtFromDriver = "localMode";
      hasError = -1;
    }
    /* Update if we have an error now.
       Update even if we had an error before - it may have gone now,
       and the we need to set the NULL pointer */
    if (hasError || drvlocal.dirty.old_hasError) {
      updateMsgTxtFromDriver(msgTxtFromDriver);
    }
    drvlocal.dirty.old_hasError = hasError;
    setIntegerParam(pC_->ethercatmcStatusCode_, idxStatusCode);
    setIntegerParam(pC_->motorStatusProblem_, drvlocal.hasProblem | localMode);
    setIntegerParamLog(pC_->motorStatusPowerOn_, powerIsOn, "powerOn");
  }

  if (pC_->ctrlLocal.systemUTCtimeOffset)  {
    /* Motor position in "user coordinates" with UTC time from PTP */
    double motorRecOffset;
    int motorRecDirection;
    asynStatus RBV_UTCstatus = asynDisabled;
    double ethercatmcRBV_UTC = 0.0;
    if (homed && positionValid && statusValid &&
        (!pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_,
                              &motorRecOffset)) &&
        (!pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_,
                               &motorRecDirection))) {

      RBV_UTCstatus = asynSuccess;
      /* direction == 1 means "negative" */
      motorRecDirection = motorRecDirection ? -1 : 1;
      ethercatmcRBV_UTC = actPosition *  motorRecDirection + motorRecOffset;
    }
    setDoubleParam(pC_->ethercatmcRBV_UTC_, ethercatmcRBV_UTC);
    pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->ethercatmcRBV_UTC_,
                                       RBV_UTCstatus);
    {
      /* encoder act in UTC */
      double ethercatmcEncAct;
      if (!pC_->getDoubleParam(axisNo_, pC_->ethercatmcEncAct_,
                               &ethercatmcEncAct)) {
        setDoubleParam(pC_->ethercatmcEncActUTC_, ethercatmcEncAct);
        pC_->setAlarmStatusSeverityWrapper(axisNo_, pC_->ethercatmcEncActUTC_,
                                           asynSuccess);
      }
    }
  }

  if ((paramCtrl & PARAM_IF_CMD_MASK) == PARAM_IF_CMD_DONE) {
    unsigned paramIndex = paramCtrl & PARAM_IF_IDX_MASK;
    if (paramIndexIsInteger(paramIndex)) {
      paramfValue = (double)paramiValue;
    }
    asynPrint(pC_->pasynUserController_,
              ASYN_TRACE_FLOW,
              "%spoll(%d) paramCtrl=%s (0x%x) paramValue=%f\n",
              modNamEMC, axisNo_,
              plcParamIndexTxtFromParamIndex(paramIndex),
              paramCtrl, paramfValue);
    if ((paramCtrl != drvlocal.old_paramCtrl) ||
        (paramfValue != drvlocal.old_paramValue)) {
      /* Only read real parameters, not functions */
      if (paramIndexIsParameterToPoll(paramIndex)) {
        int initial = 0;
        pC_->parameterFloatReadBack(axisNo_,
                                    initial,
                                    paramIndex,
                                    paramfValue);
      }
    }
    drvlocal.old_paramCtrl = paramCtrl;
    drvlocal.old_paramValue = paramfValue;
  }

  /* Read back the parameters one by one */
  if (pollReadBackInBackGround && (paramCtrl & PARAM_IF_ACK_MASK)) {
    drvlocal.pollNowIdx++;
    if (!drvlocal.pollNowParams[drvlocal.pollNowIdx]) {
      /* The list is 0 terminated */
      drvlocal.pollNowIdx = 0;
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
  drvlocal.dirty.idxStatusCode   = idxStatusCode;
  callParamCallbacks();
  return status;
}


asynStatus ethercatmcIndexerAxis::resetAxis(void)
{
  return writeCmdRegisster(idxStatusCodeRESET);
}

/** Set the motor closed loop status
 * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus ethercatmcIndexerAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynError;

  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d\n",  modNamEMC, axisNo_,
            (int)closedLoop);
  if (drvlocal.paramIfOffset) {
    double fValue = closedLoop ? 0.0 : 1.0; /* 1.0 means disable */
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    PARAM_IDX_OPMODE_AUTO_UINT,
                                    drvlocal.lenInPlcPara,
                                    fValue, NULL);
  }
  return status;
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
      memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
      drvlocal.dirty.initialPollNeeded = 1;
      drvlocal.devNum = 0;
      drvlocal.iTypCode = 0;
      drvlocal.iOffset = 0;
    }
#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modNamEMC, axisNo_, value);
#endif
  } else if (function == pC_->ethercatmcHomProc_) {
    unsigned paramIndex = PARAM_IDX_HOMPROC_UINT;
    double valueRB = -1;
    if (drvlocal.PILSparamPerm[paramIndex] != PILSparamPermWrite) {
      paramIndex = PARAM_IDX_HOMPROC_FLOAT;
    }
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ethercatmcHomProc)=%d  status=%s(%d)\n",
              modNamEMC, axisNo_, value,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else if (function == pC_->ethercatmcErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d ErrRst_)=%d\n",
                modNamEMC, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
  } else if (function == pC_->ethercatmcCfgDHLM_En_) {
    unsigned paramIndex = PARAM_IDX_USR_MAX_EN_UINT;
    double valueRB = -1;
    if (drvlocal.PILSparamPerm[paramIndex] != PILSparamPermWrite) {
      paramIndex = PARAM_IDX_USR_MAX_EN_FLOAT;
    }
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ethercatmcCfgDHLM_En)=%d  status=%s(%d)\n",
              modNamEMC, axisNo_, value,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else if (function == pC_->ethercatmcCfgDLLM_En_) {
    unsigned paramIndex = PARAM_IDX_USR_MIN_EN_UINT;
    double valueRB = -1;
    if (drvlocal.PILSparamPerm[paramIndex] != PILSparamPermWrite) {
      paramIndex = PARAM_IDX_USR_MIN_EN_FLOAT;
    }
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetIntegerParam(%d ethercatmcCfgDLLM_En)=%d status=%s(%d)\n",
              modNamEMC, axisNo_, value,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else {
    pilsAsynDevInfo_type *pPilsAsynDevInfo;
    pPilsAsynDevInfo = pC_->findIndexerOutputDevice(axisNo_, function,
                                                    asynParamInt32);
    if (pPilsAsynDevInfo) {
      const char *paramName = NULL;
      if (pC_->getParamName(axisNo_, function, &paramName)) paramName = "";

      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%ssetIntegerParam(%d %s offset=%u)=%d\n",
                modNamEMC, axisNo_,
                paramName,
                pPilsAsynDevInfo->outputOffset, value);
      status = pC_->setPlcMemoryInteger(pPilsAsynDevInfo->outputOffset,
                                        value,
                                        pPilsAsynDevInfo->lenInPLC);
      if (status == asynSuccess) {
        return asynSuccess;
      }
      return asynError;
    }
  }
  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

asynStatus ethercatmcIndexerAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  if (function == pC_->ethercatmcHomPos_) {
    static const unsigned paramIndex = PARAM_IDX_HOME_POSITION_FLOAT;
    double valueRB = -1;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ethercatmcHomPos_)=%g\n", modNamEMC, axisNo_, value);
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else if (function == pC_->ethercatmcCfgDHLM_) {
    static const unsigned paramIndex = PARAM_IDX_USR_MAX_FLOAT;
    double valueRB = -1;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ethercatmcCfgDHLM_)=%g\n", modNamEMC, axisNo_, value);
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
    if (status == asynSuccess) {
      int initial = 0;
      pC_->parameterFloatReadBack(axisNo_, initial, paramIndex, valueRB);
      return asynSuccess;
    }
    return asynError;
  } else if (function == pC_->ethercatmcCfgDLLM_) {
    double valueRB = -1;
    static const unsigned paramIndex = PARAM_IDX_USR_MIN_FLOAT;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetDoubleParam(%d ethercatmcCfgDLLM_)=%g\n", modNamEMC, axisNo_, value);
    status = pC_->indexerParamWrite(axisNo_, drvlocal.paramIfOffset,
                                    paramIndex,
                                    drvlocal.lenInPlcPara,
                                    value, &valueRB);
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
  if (function == pC_->ethercatmcDbgStrToLog_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetStringParamDbgStrToLog(%d)=\"%s\"\n",
              modNamEMC, axisNo_, value);
  } else if (function == pC_->ethercatmcDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  }
  /* Call base class method */
  return asynMotorAxis::setStringParam(function, value);
}

#ifndef motorMessageTextString
void ethercatmcIndexerAxis::updateMsgTxtFromDriver(const char *value)
{
  if (value && value[0]) {
    setStringParam(pC_->ethercatmcMCUErrMsg_,value);
  } else {
    setStringParam(pC_->ethercatmcMCUErrMsg_, "");
  }
}
#endif
