/*
  FILENAME... EthercatMCAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include <epicsThread.h>

#include "motor.h"
#include "EthercatMC.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

/* temporally definition */
#ifndef ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED
#define ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED 0x2001C
#endif

#define NCOMMANDMOVEVEL  1
#define NCOMMANDMOVEREL  2
#define NCOMMANDMOVEABS  3
#define NCOMMANDHOME    10
#define HOMPROC_MANUAL_SETPOS    15

/* The maximum number of polls we wait for the motor
   to "start" (report moving after a new move command */
#define WAITNUMPOLLSBEFOREREADY 3

const static char *const modulName = "EthercatMCAxis::";

//
// These are the EthercatMCAxis methods
//

/** Creates a new EthercatMCAxis object.
 * \param[in] pC Pointer to the EthercatMCController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
 *
 *
 * Initializes register numbers, etc.
 */
EthercatMCAxis::EthercatMCAxis(EthercatMCController *pC, int axisNo,
                     int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
#ifdef motorWaitPollsBeforeReadyString
  setIntegerParam(pC_->motorWaitPollsBeforeReady_ , WAITNUMPOLLSBEFOREREADY);
#endif
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;
  drvlocal.axisFlags = axisFlags;

  /* We pretend to have an encoder (fActPosition) */
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
#ifdef motorFlagsNoStopProblemString
  setIntegerParam(pC_->motorFlagsNoStopProblem_, 1);
#endif
#ifdef motorFlagsLSrampDownString
  setIntegerParam(pC_->motorFlagsLSrampDown_, 1);
#endif
#ifdef motorFlagsPwrWaitForOnString
  setIntegerParam(pC_->motorFlagsPwrWaitForOn_, 1);
#endif
  if (axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) {
#ifdef POWERAUTOONOFFMODE2
    setIntegerParam(pC_->motorPowerAutoOnOff_, POWERAUTOONOFFMODE2);
    setDoubleParam(pC_->motorPowerOnDelay_,   6.0);
    setDoubleParam(pC_->motorPowerOffDelay_, -1.0);
#else
    setIntegerParam(pC_->motorPowerAutoOnOff_, 1);
    setDoubleParam(pC_->motorPowerOnDelay_,   6.0);
    setDoubleParam(pC_->motorPowerOffDelay_,  4.0);
#endif
  }
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    const char * const cfgDebug_str = "getDebugText=";
    const char * const stepSize_str = "stepSize=";
    const char * const homProc_str = "HomProc=";
    const char * const homPos_str  = "HomPos=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.externalEncoderStr = strdup(pThisOption);
      }  else if (!strncmp(pThisOption, cfgfile_str, strlen(cfgfile_str))) {
        pThisOption += strlen(cfgfile_str);
        drvlocal.cfgfileStr = strdup(pThisOption);
      } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
        pThisOption += strlen(cfgDebug_str);
        drvlocal.cfgDebug_str = strdup(pThisOption);
      } else if (!strncmp(pThisOption, stepSize_str, strlen(stepSize_str))) {
        pThisOption += strlen(stepSize_str);
        double cfgStepSize = atof(pThisOption);
        if (cfgStepSize > 0.0) {
          drvlocal.cfgStepSize = cfgStepSize;
          drvlocal.stepSize = drvlocal.cfgStepSize;
        }
      } else if (!strncmp(pThisOption, homProc_str, strlen(homProc_str))) {
        pThisOption += strlen(homProc_str);
        int homProc = atoi(pThisOption);
        if (homProc < 0) homProc = 0;
        if (homProc) {
          setIntegerParam(pC_->EthercatMCHomProc_, homProc);
        }
      } else if (!strncmp(pThisOption, homPos_str, strlen(homPos_str))) {
        pThisOption += strlen(homPos_str);
        double homPos = atof(pThisOption);
        setDoubleParam(pC_->EthercatMCHomPos_, homPos);
      }
      pThisOption = pNextOption;
    }
    free(pOptions);
  }
}


extern "C" int EthercatMCCreateAxis(const char *EthercatMCName, int axisNo,
                               int axisFlags, const char *axisOptionsStr)
{
  EthercatMCController *pC;

  pC = (EthercatMCController*) findAsynPortDriver(EthercatMCName);
  if (!pC)
  {
    printf("Error port %s not found\n", EthercatMCName);
    return asynError;
  }
  pC->lock();
  new EthercatMCAxis(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
void EthercatMCAxis::handleDisconnect(asynStatus status)
{
  if (status != asynSuccess) {
    memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
    drvlocal.MCU_nErrorId = 0;
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacksUpdateError();
    drvlocal.dirty.initialPollNeeded = 1;
  }
}


asynStatus EthercatMCAxis::readBackSoftLimits(void)
{
  asynStatus status;
  int iValueHigh = 0, iValueLow = 0;
  double fValueHigh = 0.0, fValueLow  = 0.0;
  double stepSize = drvlocal.stepSize;

  /* High limits Low Enable */
  status = getSAFValuesFromAxisPrint(0x5000, 0xC, "CHLM_En", &iValueHigh,
                                     0x5000, 0xE, "CHLM", &fValueHigh);
  if (status == asynSuccess) {
    /* Low limits Low Enable */
    status = getSAFValuesFromAxisPrint(0x5000, 0xB, "CLLM_En", &iValueLow,
                                       0x5000, 0xD, "CLLM", &fValueLow);
  }
  if (status != asynSuccess) {
    /* Communication problem, set everything to 0 */
    iValueHigh = iValueLow = 0;
    fValueHigh = fValueLow = 0.0;
  }
  /* EthercatMCCHLMXX are info(asyn:READBACK,"1"),
     so we must use pC_->setXXX(axisNo_..)  here */
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCHLM_En_, iValueHigh);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCHLM_, fValueHigh);
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCCLLM_En_, iValueLow);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCCLLM_, fValueLow);

  if (!iValueHigh || !iValueLow || !stepSize || fValueLow >= fValueHigh) {
    /* Any limit not active, or stepSize == 0.0
       Set everything to 0 */
    fValueHigh = fValueLow  = 0.0;
    /* avoid dividing by 0 */
    if (!stepSize) stepSize = 1.0;
  }
#ifdef motorHighLimitROString
  setDoubleParam(pC_->motorHighLimitRO_, fValueHigh / stepSize);
  setDoubleParam(pC_->motorLowLimitRO_, fValueLow / stepSize);
#endif
  return status;
}

asynStatus EthercatMCAxis::readBackHoming(void)
{
  asynStatus status;
  int    homProc = 0;
  double homPos  = 0.0;

  /* Don't read it, when the driver has been configured with HomProc= */
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&homProc);
  if (status == asynSuccess) return status;

  status = getValueFromAxis("_EPICS_HOMPROC", &homProc);
  if (!status) setIntegerParam(pC_->EthercatMCHomProc_, homProc);
  status = getValueFromAxis("_EPICS_HOMPOS", &homPos);
  if (status) {
    /* fall back */
    status = getSAFValueFromAxisPrint(0x5000, 0x103, "homPos", &homPos);
  }
  if (!status) pC_->setDoubleParam(axisNo_, pC_->EthercatMCHomPos_, homPos);
  return asynSuccess;
}


/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxis::readBackConfig(void)
{
  asynStatus status;
  int iValue;
  double fValue;
  double stepSize = drvlocal.stepSize;

  if (!stepSize) return asynError;
  /* (Micro) steps per revolution */
  status = getSAFValueFromAxisPrint(0x5000, 0x24, "SREV", &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalSREV_RB_, fValue);

  /* EGU per revolution */
  status = getSAFValueFromAxisPrint(0x5000, 0x23, "UREV", &fValue);
  if (status == asynSuccess) {
    /* mres is urev/srev */
    double srev;
    setDoubleParam(pC_->EthercatMCScalUREV_RB_, fValue);
    pC_->getDoubleParam(axisNo_, pC_->EthercatMCScalSREV_RB_, &srev);
#ifdef motorSDBDROString
    if (srev)
      setDoubleParam(pC_->motorSDBDRO_, fValue / srev);
#endif
  }

  /* Reference Velocity */
  status = getSAFValueFromAxisPrint(0x7000, 0x101, "RefVelo", &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalRefVelo_RB_, fValue);
  /* Motor DIRection */
  status = getSAFValueFromAxisPrint(0x7000, 0x6, "MDIR", &iValue);
  if (status == asynSuccess) setIntegerParam(pC_->EthercatMCScalMDIR_RB_, iValue);
  /* Encoder DIRection */
  status = getSAFValueFromAxisPrint(0x5000, 0x8, "EDIR", &iValue);
  if (status == asynSuccess) setIntegerParam(pC_->EthercatMCScalEDIR_RB_, iValue);

  /* In target position monitor window and enable*/
  status = getSAFValuesFromAxisPrint(0x4000, 0x15,"RDBD_En", &iValue,
                                     0x4000, 0x16,"RDBD_RB", &fValue);

  if (status == asynSuccess) {
    setDoubleParam(pC_->EthercatMCScalRDBD_RB_, fValue);
    setIntegerParam(pC_->EthercatMCScalRDBD_En_RB_, iValue);
#ifdef motorRDBDROString
    setDoubleParam(pC_->motorRDBDRO_, iValue ? fValue : 0.0);
#endif
  }
  /* In target position monitor time */
  status = getSAFValueFromAxisPrint(0x4000, 0x17, "RDBD_Tim", &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalRDBD_Tim_RB_,
                                            fValue);

  readBackSoftLimits();
  readBackHoming();
  /* The Ethercat specific are  read-write, so we must use pC_->setXXX for them */
  /* (fast) Velocity */
  status = getSAFValueFromAxisPrint(0x4000, 0x9, "VELO", &fValue);
  if (status == asynSuccess) pC_->setDoubleParam(axisNo_, pC_->EthercatMCCFGVELO_, fValue / stepSize);
#ifdef motorDefVelocityROString
  if (status == asynSuccess) setDoubleParam(pC_->motorDefVelocityRO_, fValue / stepSize);
#endif

  /* Maximal Velocity */
  status = getSAFValueFromAxisPrint(0x4000, 0x27, "VMAX", &fValue);
  if (status == asynSuccess) pC_->setDoubleParam(axisNo_, pC_->EthercatMCCFGVMAX_, fValue / stepSize);
#ifdef motorMaxVelocityROString
  if (status == asynSuccess) setDoubleParam(pC_->motorMaxVelocityRO_, fValue / stepSize);
#endif

  /* (slow) Velocity */
  status = getSAFValueFromAxisPrint(0x4000, 0x8, "JVEL", &fValue);
  if (status == asynSuccess) pC_->setDoubleParam(axisNo_, pC_->EthercatMCCFGJVEL_, fValue / stepSize);
#ifdef motorDefJogVeloROString
  if (status == asynSuccess) setDoubleParam(pC_->motorDefJogVeloRO_, fValue / stepSize);
#endif

  /* (default) Acceleration */
  status = getSAFValueFromAxisPrint(0x4000, 0x101, "JAR", &fValue);
  if (status == asynSuccess) pC_->setDoubleParam(axisNo_, pC_->EthercatMCCFGJAR_, fValue / stepSize);
#ifdef motorDefJogAccROString
  if (status == asynSuccess) setDoubleParam(pC_->motorDefJogAccRO_, fValue / stepSize);
#endif
  return status;
}


/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxis::initialPoll(void)
{
  asynStatus status = asynSuccess;

  /*  Check for Axis ID */
  int axisID = getMotionAxisID();
  if (axisID  == -1) {
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    return asynError;
  } else if (axisID  == -2) {
    updateMsgTxtFromDriver("No AxisID");
    return asynSuccess;
  }
  if (axisID != axisNo_) {
    updateMsgTxtFromDriver("ConfigError AxisID");
    return asynError;
  }
  status = getFeatures();
  if (status) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s (%d) getFeatures() failed\n",
               modulName, axisNo_);
    updateMsgTxtFromDriver("getFeatures() failed");
    return status;
  }
  status = readConfigFile();
  if (status) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s (%d) readConfigFile() failed\n",
               modulName, axisNo_);
    updateMsgTxtFromDriver("ConfigError Config File");
    return status;
  }
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }
  if (status == asynSuccess && !drvlocal.supported.bECMC) {
    /* for ECMC everything is configured from EPICS, do NOT do the readback */
    status = readBackConfig();
  }
  if (status == asynSuccess && drvlocal.dirty.oldStatusDisconnected) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
	      "%s connected(%d)\n",  modulName, axisNo_);
    drvlocal.dirty.oldStatusDisconnected = 0;
  }
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void EthercatMCAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


extern "C" const char *errStringFromErrId(int nErrorId)
{
  switch(nErrorId) {
  case 0x4221:
    return "Velocity not allowed";
  case 0x4223:
    return "Axis positioning enable";
  case 0x4450:
  case 0x4451:
    return "Following error";
  case 0x4260:
    return "Amplifier off";
  case 0x4263:
    return "Is still being processed";
  case 0x4460:
    return "Low soft limit";
  case 0x4461:
    return "High soft limit";
  case 0x4550:
    return "Following err mon pos";
  case 0x4551:
    return "Following err mon vel";
  case 0x4655:
    return "Invalid IO data";
  case 0x4B0A:
    return "Homing not successful or not started";
  default:
    return "Controller error";
  }
}




/** Set velocity and acceleration for the axis
 * \param[in] maxVelocity, mm/sec
 * \param[in] acceleration ???
 *
 */
asynStatus EthercatMCAxis::sendVelocityAndAccelExecute(double maxVelocity, double acceleration_time)
{
  asynStatus status;
  /* We don't use minVelocity */
  double maxVelocityEGU = maxVelocity * drvlocal.stepSize;
  if (!drvlocal.stepSize) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s sendVelocityAndAccelExecute(%d) stepSize==0.0\n",
              modulName, axisNo_);
    return asynError; /* No stepSize, no move */
  }
  if (acceleration_time > 0.0001) {
    double acc_in_seconds = maxVelocity / acceleration_time;
    double acc_in_EGU_sec2 = maxVelocityEGU / acc_in_seconds;
    if (acc_in_EGU_sec2  < 0) acc_in_EGU_sec2 = 0 - acc_in_EGU_sec2 ;
    status = setValuesOnAxis("fAcceleration", acc_in_EGU_sec2,
                             "fDeceleration", acc_in_EGU_sec2);
    if (status) return status;
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s sendVelocityAndAccelExecute(%d) maxVelocityEGU=%g acceleration_time=%g\n",
               modulName, axisNo_, maxVelocityEGU, acceleration_time);
  }
  status = setValueOnAxis("fVelocity", maxVelocityEGU);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif
  return status;
}

/** Move the axis to a position, either absolute or relative
 * \param[in] position in mm
 * \param[in] relative (0=absolute, otherwise relative)
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  int nCommand = relative ? NCOMMANDMOVEREL : NCOMMANDMOVEABS;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = setValueOnAxis("fPosition", position * drvlocal.stepSize);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}


/** Home the motor, search the home position
 * \param[in] minimum velocity, mm/sec
 * \param[in] maximum velocity, mm/sec
 * \param[in] acceleration, seconds to maximum velocity
 * \param[in] forwards (0=backwards, otherwise forwards)
 *
 */
asynStatus EthercatMCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  int nCommand = NCOMMANDHOME;

  int homProc = -1;
  double homPos = 0.0;

  /* The homPos may be undefined, then use 0.0 */
  (void)pC_->getDoubleParam(axisNo_, pC_->EthercatMCHomPos_, &homPos);
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_,&homProc);
  if (homProc == HOMPROC_MANUAL_SETPOS)
    return asynError;
  /* The controller will do the home search, and change its internal
     raw value to what we specified in fPosition. */
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("fHomePosition", homPos);


  if (drvlocal.supported.bECMC) {
    double velToHom;
    double velFrmHom;
    double accHom;
    double decHom;
    if (!status) status = pC_->getDoubleParam(axisNo_,
					      pC_->EthercatMCVelToHom_,
					      &velToHom);
    if (!status) status = pC_->getDoubleParam(axisNo_,
					      pC_->EthercatMCVelFrmHom_,
					      &velFrmHom);
    if (!status) status = pC_->getDoubleParam(axisNo_,
					      pC_->EthercatMCAccHom_,
					      &accHom);
    if (!status) status = pC_->getDoubleParam(axisNo_,
					      pC_->EthercatMCDecHom_,
					      &decHom);
    if (!status) status = setSAFValueOnAxis(0x4000, 0x6,
					    velToHom);
    if (!status) status = setSAFValueOnAxis(0x4000, 0x7,
					    velFrmHom);
    if (!status)  status = setValuesOnAxis("fAcceleration", accHom,
					   "fDeceleration", decHom);
  }
  if (!status) status = setValueOnAxis("nCommand", nCommand );
  if (!status) status = setValueOnAxis("nCmdData", homProc);

  if (!status) status = setValueOnAxis("bExecute", 1);
#ifndef motorWaitPollsBeforeReadyString
  drvlocal.waitNumPollsBeforeReady += WAITNUMPOLLSBEFOREREADY;
#endif
  return status;
}


/** jog the the motor, search the home position
 * \param[in] minimum velocity, mm/sec (not used)
 * \param[in] maximum velocity, mm/sec (positive or negative)
 * \param[in] acceleration, seconds to maximum velocity
 *
 */
asynStatus EthercatMCAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;

  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) setValueOnAxis("nCommand", NCOMMANDMOVEVEL);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}



/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCAxis::setPosition(double value)
{
  asynStatus status = asynSuccess;
  int nCommand = NCOMMANDHOME;
  int homProc = 0;
  double homPos = value;

  status = pC_->getIntegerParam(axisNo_,
                                pC_->EthercatMCHomProc_,
                                &homProc);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s setPosition(%d  homProc=%d position=%g egu=%g\n",
            modulName, axisNo_,  homProc, value, value * drvlocal.stepSize );

  if (homProc != HOMPROC_MANUAL_SETPOS)
    return asynError;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("fHomePosition", homPos);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand );
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", homProc);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);

  return status;
}

/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus EthercatMCAxis::setHighLimit(double highLimit)
{
  drvlocal.motorRecordHighLimit = highLimit;
  return asynSuccess;
}


/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus EthercatMCAxis::setLowLimit(double lowLimit)
{
  drvlocal.motorRecordLowLimit = lowLimit;
  return asynSuccess;
}


asynStatus EthercatMCAxis::resetAxis(void)
{
  asynStatus status = asynSuccess;
  int EthercatMCErr;
  bool moving;
  /* Reset command error, if any */
  drvlocal.cmdErrorMessage[0] = 0;
  status = pC_->getIntegerParam(axisNo_, pC_->EthercatMCErr_, &EthercatMCErr);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s resetAxis(%d status=%d EthercatMCErr)=%d\n",
             modulName, axisNo_, (int)status, EthercatMCErr);

  if (EthercatMCErr) {
    /* Soft reset of the axis */
    status = setValueOnAxis("bExecute", 0);
    if (status) goto resetAxisReturn;
    status = setValueOnAxisVerify("bReset", "bReset", 1, 20);
    if (status) goto resetAxisReturn;
    epicsThreadSleep(.1);
    status = setValueOnAxisVerify("bReset", "bReset", 0, 20);
  }
  resetAxisReturn:
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s resetAxis(%d) status=%s (%d)\n",
             modulName, axisNo_, pasynManager->strStatus(status), (int)status);
  /* do a poll */
  poll(&moving);
  return status;
}

bool EthercatMCAxis::pollPowerIsOn(void)
{
  int ret = 0;
  asynStatus status = getValueFromAxis(".bEnabled", &ret);
  if (!status && ret)
    return true;
  else
    return false;
}

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCAxis::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;
  bool moving;
  int ret;
  const char *enableEnabledReadback = "bEnabled";

#ifdef POWERAUTOONOFFMODE2
  {
    int autoPower;
    pC_->getIntegerParam(axisNo_, pC_->motorPowerAutoOnOff_, &autoPower);
    if (autoPower) {
      /* The record/driver will check for enabled - don't do that here */
      enableEnabledReadback = "bEnable";
    }
  }
#endif
  on = on ? 1 : 0; /* either 0 or 1 */
  status = getValueFromAxis(".bEnabled", &ret);
  /* Either it went wrong OR the amplifier IS as it should be */
  if (status || (ret == on)) return status;
  if (!on) {
    /* Amplifier is on and should be turned off.
       Stop the axis by setting bEnable to 0 */
    status = stopAxisInternal(__FUNCTION__, 0);
    if (status) return status;
  }
  status = setValueOnAxis("bEnable", on);
  if (status || !on) return status; /* this went wrong OR it should be turned off */
  while (counter) {
    epicsThreadSleep(.1);
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s?;%sMain.M%d.%s?",
             drvlocal.adsport_str, axisNo_, "bBusy",
             drvlocal.adsport_str, axisNo_, enableEnabledReadback);
    status = pC_->writeReadController();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s out=%s in=%s status=%s (%d)\n",
               modulName, pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);
    if (status) return status;
    if (!strcmp("0;1", pC_->inString_)) {
      /* bBusy == 0; bEnable(d) == 1 */
      goto enableAmplifierPollAndReturn;
    } else if (!strcmp("1;1", pC_->inString_)) {
      /* bBusy=1 is OK */
      goto enableAmplifierPollAndReturn;
    }
    counter--;
  }
  /* if we come here, it went wrong */
  if (!drvlocal.cmdErrorMessage[0]) {
    snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
             "E: enableAmplifier(%d) failed. out=%s in=%s\n",
             axisNo_, pC_->outString_, pC_->inString_);
    /* The poller co-ordinates the writing into the parameter library */
  }
enableAmplifierPollAndReturn:
  poll(&moving);
  return status;

}

/** Stop the axis
 *
 */
asynStatus EthercatMCAxis::stopAxisInternal(const char *function_name, double acceleration)
{
  asynStatus status;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s stopAxisInternal(%d) (%s)\n",   modulName, axisNo_, function_name);
  status = setValueOnAxisVerify("bExecute", "bExecute", 0, 1);
  return status;
}

/** Stop the axis, called by motor Record
 *
 */
asynStatus EthercatMCAxis::stop(double acceleration )
{
  return stopAxisInternal(__FUNCTION__, acceleration);
}

void EthercatMCAxis::callParamCallbacksUpdateError()
{
  int EPICS_nErrorId = drvlocal.MCU_nErrorId;
  drvlocal.eeAxisError = eeAxisErrorNoError;
  if (drvlocal.supported.statusVer == -1) {
    drvlocal.eeAxisError = eeAxisErrorNotFound;
  } else if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
  } else if (!drvlocal.homed &&
             (drvlocal.nCommandActive != NCOMMANDHOME) &&
             (drvlocal.motorRecordHighLimit > drvlocal.motorRecordLowLimit)) {
    int homProc;
    pC_->getIntegerParam(axisNo_, pC_->EthercatMCHomProc_, &homProc);
    if (homProc) drvlocal.eeAxisError = eeAxisErrorNotHomed;
  }
  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId ||
      drvlocal.old_nCommandActive != drvlocal.nCommandActive) {

    if (!drvlocal.cfgDebug_str) {
      if (!EPICS_nErrorId)
        updateMsgTxtFromDriver(NULL);

      switch (drvlocal.eeAxisError) {
        case eeAxisErrorNoError:
          {
            switch(drvlocal.nCommandActive) {
            case NCOMMANDMOVEVEL:
              updateMsgTxtFromDriver("I: Moving VEL");
              break;
            case NCOMMANDMOVEREL:
              updateMsgTxtFromDriver("I: Moving REL");
              break;
            case NCOMMANDMOVEABS:
              updateMsgTxtFromDriver("I: Moving ABS");
              break;
            case NCOMMANDHOME:
              updateMsgTxtFromDriver("I: Homing");
              break;
            case 0:
              updateMsgTxtFromDriver(NULL);
              break;
            default:
              updateMsgTxtFromDriver("I: Moving");
            }
          }
          break;
      case eeAxisErrorNotFound:
          updateMsgTxtFromDriver("Not found");
          break;
      case eeAxisErrorCmdError:
          updateMsgTxtFromDriver(drvlocal.cmdErrorMessage);
          break;
        case eeAxisErrorIOCcomError:
        case eeAxisErrorNotHomed:
          /* handled by asynMotorAxis, fall through */
        default:
          ;
      }
    }
    /* Axis has a problem: Report to motor record */
    setIntegerParam(pC_->motorStatusProblem_,
                    drvlocal.eeAxisError != eeAxisErrorNoError);

    /* MCU has a problem: set the red light in CSS */
    setIntegerParam(pC_->EthercatMCErr_,
                    drvlocal.eeAxisError == eeAxisErrorMCUError);
    setIntegerParam(pC_->EthercatMCErrId_, EPICS_nErrorId);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) callParamCallbacksUpdateError eeAxisError=%d old=%d ErrID=0x%x old=0x%x nCmd=%d old=%d\n",
               modulName, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId,
              drvlocal.nCommandActive, drvlocal.old_nCommandActive);

    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
    drvlocal.old_nCommandActive = drvlocal.nCommandActive;
  }

  callParamCallbacks();
}


asynStatus EthercatMCAxis::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
{
  asynStatus comStatus;

  int motor_axis_no = 0;
  int nvals = 0;
  const char * const Main_dot_str = "Main.";
  const size_t       Main_dot_len = strlen(Main_dot_str);
  struct {
    double velocitySetpoint;
    int cycleCounter;
    unsigned int EtherCATtime_low32;
    unsigned int EtherCATtime_high32;
    int command;
    int cmdData;
    int reset;
    int moving;
    int stall;
  } notUsed;
  if (drvlocal.dirty.initialPollNeeded) {
    comStatus = initialPoll();
    if (comStatus) return comStatus;
    drvlocal.dirty.initialPollNeeded = 0;
  }

  if (drvlocal.supported.stAxisStatus_V2 || drvlocal.dirty.stAxisStatus_Vxx) {
    /* V2 is supported, use it. Or. unkown: try it as well */
    snprintf(pC_->outString_, sizeof(pC_->outString_),
            "%sMain.M%d.stAxisStatusV2?", drvlocal.adsport_str, axisNo_);
    comStatus = pC_->writeReadController();
    if (!strncasecmp(pC_->inString_,  Main_dot_str, Main_dot_len)) {
      nvals = sscanf(&pC_->inString_[Main_dot_len],
                     "M%d.stAxisStatusV2="
                     "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     &motor_axis_no,
                     &pst_axis_status->fPosition,
                     &pst_axis_status->fActPosition,
                     &pst_axis_status->positionRaw,          /* Send as uint64; parsed as double ! */
                     &notUsed.velocitySetpoint,
                     &pst_axis_status->fActVelocity,
                     &pst_axis_status->fAcceleration,
                     &pst_axis_status->fDecceleration,
                     &notUsed.cycleCounter,
                     &notUsed.EtherCATtime_low32,
                     &notUsed.EtherCATtime_high32,
                     &pst_axis_status->bEnable,
                     &pst_axis_status->bEnabled,
                     &pst_axis_status->bExecute,
                     &notUsed.command,
                     &notUsed.cmdData,
                     &pst_axis_status->bLimitBwd,
                     &pst_axis_status->bLimitFwd,
                     &pst_axis_status->bHomeSensor,
                     &pst_axis_status->bError,
                     &pst_axis_status->nErrorId,
                     &notUsed.reset,
                     &pst_axis_status->bHomed,
                     &pst_axis_status->bBusy,
                     &pst_axis_status->atTarget,
                     &notUsed.moving,
                     &notUsed.stall);
    }
    if (nvals == 27) {
      if (drvlocal.dirty.stAxisStatus_Vxx) {
        drvlocal.supported.stAxisStatus_V2 = 1;
      }
      pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy || !pst_axis_status->atTarget;
    }
  }
  if (!drvlocal.supported.stAxisStatus_V2) {
    /* Read the complete Axis status */
    snprintf(pC_->outString_, sizeof(pC_->outString_),
            "%sMain.M%d.stAxisStatus?", drvlocal.adsport_str, axisNo_);
    comStatus = pC_->writeReadController();
    if (comStatus) return comStatus;
    if (!strncasecmp(pC_->inString_,  Main_dot_str, Main_dot_len)) {
      nvals = sscanf(&pC_->inString_[Main_dot_len],
                     "M%d.stAxisStatus="
                     "%d,%d,%d,%u,%u,%lf,%lf,%lf,%lf,%d,"
                     "%d,%d,%d,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d",
                     &motor_axis_no,
                     &pst_axis_status->bEnable,        /*  1 */
                     &pst_axis_status->bReset,         /*  2 */
                     &pst_axis_status->bExecute,       /*  3 */
                     &pst_axis_status->nCommand,       /*  4 */
                     &pst_axis_status->nCmdData,       /*  5 */
                     &pst_axis_status->fVelocity,      /*  6 */
                     &pst_axis_status->fPosition,      /*  7 */
                     &pst_axis_status->fAcceleration,  /*  8 */
                     &pst_axis_status->fDecceleration, /*  9 */
                     &pst_axis_status->bJogFwd,        /* 10 */
                     &pst_axis_status->bJogBwd,        /* 11 */
                     &pst_axis_status->bLimitFwd,      /* 12 */
                     &pst_axis_status->bLimitBwd,      /* 13 */
                     &pst_axis_status->fOverride,      /* 14 */
                     &pst_axis_status->bHomeSensor,    /* 15 */
                     &pst_axis_status->bEnabled,       /* 16 */
                     &pst_axis_status->bError,         /* 17 */
                     &pst_axis_status->nErrorId,       /* 18 */
                     &pst_axis_status->fActVelocity,   /* 19 */
                     &pst_axis_status->fActPosition,   /* 20 */
                     &pst_axis_status->fActDiff,       /* 21 */
                     &pst_axis_status->bHomed,         /* 22 */
                     &pst_axis_status->bBusy           /* 23 */);
    }
    if (nvals != 24) {
      drvlocal.supported.stAxisStatus_V1 = 0;
      goto pollAllWrongnvals;
    }
    drvlocal.supported.stAxisStatus_V1 = 1;

    /* V1 new style: mvnNRdyNex follows bBusy */
    if (drvlocal.supported.bSIM || drvlocal.supported.bECMC)
      drvlocal.supported.bV1BusyNewStyle = 1;

    pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy && pst_axis_status->bEnabled;
    if (!drvlocal.supported.bV1BusyNewStyle) {
      /* "V1 old style":done when bEcecute is 0 */
      pst_axis_status->mvnNRdyNex &= pst_axis_status->bExecute;
    }
  } /* End of V1 */
  /* From here on, either V1 or V2 is supported */
  if (drvlocal.dirty.stAxisStatus_Vxx) {
    if (drvlocal.supported.stAxisStatus_V2)
      drvlocal.supported.statusVer = 2;
    else if (drvlocal.supported.stAxisStatus_V1 && !drvlocal.supported.bV1BusyNewStyle)
      drvlocal.supported.statusVer = 0;
    else if (drvlocal.supported.stAxisStatus_V1 && drvlocal.supported.bV1BusyNewStyle)
      drvlocal.supported.statusVer = 1;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s pollAll(%d) nvals=%d V1=%d V2=%d sim=%d ecmc=%d bV1BusyNewStyle=%d Ver=%d fActPosition=%f\n",
              modulName, axisNo_, nvals,
              drvlocal.supported.stAxisStatus_V1,
              drvlocal.supported.stAxisStatus_V2,
              drvlocal.supported.bSIM,
              drvlocal.supported.bECMC,
              drvlocal.supported.bV1BusyNewStyle,
              drvlocal.supported.statusVer,
              pst_axis_status->fActPosition);
#ifdef motorFlagsHomeOnLsString
    setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);
#endif
#ifdef motorFlagsStopOnProblemString
    setIntegerParam(pC_->motorFlagsStopOnProblem_, 0);
#endif
    drvlocal.dirty.stAxisStatus_Vxx = 0;
  }
  if (axisNo_ != motor_axis_no) return asynError;

  /* Use previous fActPosition and current fActPosition to calculate direction.*/
  if (pst_axis_status->fActPosition > drvlocal.old_st_axis_status.fActPosition) {
    pst_axis_status->motorDiffPostion = 1;
    pst_axis_status->motorStatusDirection = 1;
  } else if (pst_axis_status->fActPosition < drvlocal.old_st_axis_status.fActPosition) {
    pst_axis_status->motorDiffPostion = 1;
    pst_axis_status->motorStatusDirection = 0;
  }
  if (!pst_axis_status->bEnabled) {
    /* if the motor is moved by with amplifier off, report this */
    pst_axis_status->mvnNRdyNex |= pst_axis_status->motorDiffPostion;
  }
  return asynSuccess;


pollAllWrongnvals:
  drvlocal.supported.statusVer = -1;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s pollAll(%d) nvals=%d in=%s\n",
            modulName, axisNo_, nvals, pC_->inString_);
  return asynSuccess;
}


/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus EthercatMCAxis::poll(bool *moving)
{
  asynStatus comStatus = asynSuccess;
  st_axis_status_type st_axis_status;

  if (drvlocal.supported.statusVer == -1) {
    callParamCallbacksUpdateError();
    return asynSuccess;
  }
  /* Driver not yet initialized, do nothing */
  if (!drvlocal.stepSize) return comStatus;

  memset(&st_axis_status, 0, sizeof(st_axis_status));
  comStatus = pollAll(moving, &st_axis_status);
  if (comStatus) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s out=%s in=%s return=%s (%d)\n",
               modulName, pC_->outString_, pC_->inString_,
              pasynManager->strStatus(comStatus), (int)comStatus);
    goto skip;
  }

  if (drvlocal.cfgDebug_str) {
    asynStatus comStatus;
    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadController();
    if (!comStatus) {
      updateMsgTxtFromDriver(pC_->inString_);
    }
  }

  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  drvlocal.homed = st_axis_status.bHomed;
  setIntegerParam(pC_->motorStatusCommsError_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);
  setDoubleParam(pC_->EthercatMCVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->EthercatMCAcc_RB_, st_axis_status.fAcceleration);
  setDoubleParam(pC_->EthercatMCDec_RB_, st_axis_status.fDecceleration);

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  }
  else
#endif
  {
    *moving = st_axis_status.mvnNRdyNex ? true : false;
  }

  if (st_axis_status.mvnNRdyNex)
    drvlocal.nCommandActive = st_axis_status.nCommand;
  else
    drvlocal.nCommandActive = 0;

  if (drvlocal.nCommandActive != NCOMMANDHOME) {
    double newPositionInSteps = st_axis_status.fActPosition / drvlocal.stepSize;
    setDoubleParam(pC_->motorPosition_, newPositionInSteps);
    setDoubleParam(pC_->motorEncoderPosition_, newPositionInSteps);
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
    setDoubleParam(pC_->EthercatMCVel_RB_, st_axis_status.fVelocity);
  }

  if (drvlocal.externalEncoderStr) {
    comStatus = getValueFromController(drvlocal.externalEncoderStr,
                                       &st_axis_status.positionRaw);
    if (!comStatus) setDoubleParam(pC_->EthercatMCEncAct_,
                                   st_axis_status.positionRaw);
  } else if (drvlocal.supported.stAxisStatus_V2) {
    setDoubleParam(pC_->EthercatMCEncAct_, st_axis_status.positionRaw);
  }

  if (drvlocal.old_st_axis_status.bHomed != st_axis_status.bHomed) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) homed=%d\n",
              modulName, axisNo_, st_axis_status.bHomed);
    drvlocal.old_st_axis_status.bHomed =  st_axis_status.bHomed;
  }
  if (drvlocal.old_st_axis_status.bLimitBwd != st_axis_status.bLimitBwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) LLS=%d\n",
              modulName, axisNo_, !st_axis_status.bLimitBwd);
    drvlocal.old_st_axis_status.bLimitBwd =  st_axis_status.bLimitBwd;
  }
  if (drvlocal.old_st_axis_status.bLimitFwd != st_axis_status.bLimitFwd) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) HLS=%d\n",
              modulName, axisNo_,!st_axis_status.bLimitFwd);
    drvlocal.old_st_axis_status.bLimitFwd = st_axis_status.bLimitFwd;
  }

#ifndef motorWaitPollsBeforeReadyString
  if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex) {
    drvlocal.waitNumPollsBeforeReady = 0;
  }
  if (drvlocal.waitNumPollsBeforeReady) {
    /* Don't update moving, done, motorStatusProblem_ */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) mvnNRdyNexAt=%d Ver=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d waitNumPollsBeforeReady=%d\n",
               modulName,
              axisNo_, st_axis_status.mvnNRdyNex,
              drvlocal.supported.statusVer,
              st_axis_status.bBusy, st_axis_status.bExecute,
              st_axis_status.bEnabled, st_axis_status.atTarget,
              drvlocal.waitNumPollsBeforeReady);
    drvlocal.waitNumPollsBeforeReady--;
    callParamCallbacks();
  }
  else
#endif
  {
    if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex ||
        drvlocal.old_st_axis_status.bBusy      != st_axis_status.bBusy ||
        drvlocal.old_st_axis_status.bEnabled   != st_axis_status.bEnabled ||
        drvlocal.old_st_axis_status.bExecute   != st_axis_status.bExecute ||
        drvlocal.old_st_axis_status.atTarget   != st_axis_status.atTarget) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s poll(%d) mvnNRdyNexAt=%d Ver=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d ENC=%g fPosition=%g fActPosition=%g\n",
                modulName, axisNo_, st_axis_status.mvnNRdyNex,
                drvlocal.supported.statusVer,
                st_axis_status.bBusy, st_axis_status.bExecute,
                st_axis_status.bEnabled, st_axis_status.atTarget,
                st_axis_status.positionRaw, st_axis_status.fPosition,
                st_axis_status.fActPosition);
    }
  }
  setIntegerParam(pC_->motorStatusDirection_, st_axis_status.motorStatusDirection);
  setIntegerParam(pC_->motorStatusMoving_, st_axis_status.mvnNRdyNex);
  setIntegerParam(pC_->motorStatusDone_, !st_axis_status.mvnNRdyNex);

  drvlocal.MCU_nErrorId = st_axis_status.nErrorId;

  if (drvlocal.cfgDebug_str) {
    ; /* Do not do the following */
  } else if (drvlocal.old_bError != st_axis_status.bError ||
      drvlocal.old_MCU_nErrorId != drvlocal.MCU_nErrorId ||
      drvlocal.dirty.sErrorMessage) {
    char sErrorMessage[256];
    int nErrorId = st_axis_status.nErrorId;
    memset(&sErrorMessage[0], 0, sizeof(sErrorMessage));
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s poll(%d) bError=%d st_axis_status.nErrorId=0x%x\n",
               modulName, axisNo_, st_axis_status.bError,
              nErrorId);
    drvlocal.old_bError = st_axis_status.bError;
    drvlocal.old_MCU_nErrorId = nErrorId;
    drvlocal.dirty.sErrorMessage = 0;
    snprintf(sErrorMessage, sizeof(sErrorMessage)-1, "E: %s %x",
             errStringFromErrId(nErrorId), nErrorId);
    if (sErrorMessage[0]) {
      updateMsgTxtFromDriver(sErrorMessage);
    } else if (!sErrorMessage[0] && nErrorId) {
      asynStatus status;
      status = getStringFromAxis("sErrorMessage", (char *)&sErrorMessage[0], sizeof(sErrorMessage));

      if (status == asynSuccess) updateMsgTxtFromDriver(sErrorMessage);
    }
  }
  callParamCallbacksUpdateError();

  memcpy(&drvlocal.old_st_axis_status, &st_axis_status,
         sizeof(drvlocal.old_st_axis_status));
  return asynSuccess;

  skip:
  handleDisconnect(asynError);
  return asynError;
}

/** Set the motor closed loop status
  * \param[in] closedLoop true = close loop, false = open looop. */
asynStatus EthercatMCAxis::setClosedLoop(bool closedLoop)
{
  int value = closedLoop ? 1 : 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s setClosedLoop(%d)=%d\n",  modulName, axisNo_, value);
  if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    return enableAmplifier(value);
  }
  return asynSuccess;
}

asynStatus EthercatMCAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;
  if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d motorUpdateStatus_)=%d\n", modulName, axisNo_, value);
#ifdef motorRecDirectionString
  } else if (function == pC_->motorRecDirection_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d motorRecDirection_)=%d\n",
              modulName, axisNo_, value);
#endif
#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d motorPowerAutoOnOff_)=%d\n", modulName, axisNo_, value);
#endif
#ifdef EthercatMCHomProcString
  } else if (function == pC_->EthercatMCHomProc_) {
    int motorNotHomedProblem = 0;
#ifdef  motorNotHomedProblemString
    /* If value != 0 the axis can be homed. Show Error if it isn't homed */
    if (value) motorNotHomedProblem = MOTORNOTHOMEDPROBLEM_ERROR;
    setIntegerParam(pC_->motorNotHomedProblem_, motorNotHomedProblem);
#endif
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d HomProc_)=%d motorNotHomedProblem=%d\n",
              modulName, axisNo_, value, motorNotHomedProblem);
#endif
#ifdef EthercatMCErrRstString
  } else if (function == pC_->EthercatMCErrRst_) {
    if (value) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s setIntegerParam(%d ErrRst_)=%d\n",
                modulName, axisNo_, value);
      /*  We do not want to call the base class */
      return resetAxis();
    }
    /* If someone writes 0 to the field, just ignore it */
    return asynSuccess;
#endif
#ifdef EthercatMCCHLM_EnString
  } else if (function == pC_->EthercatMCCHLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d EthercatMCCHLM_En)=%d\n",
              modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xC, value);
    readBackSoftLimits();
    return status;
#endif
#ifdef EthercatMCCLLM_EnString
  } else if (function == pC_->EthercatMCCLLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d EthercatMCCLLM_En)=%d\n",
              modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xB, value);
    readBackSoftLimits();
    return status;
#endif
  }

  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
 * \param[in] function, which parameter is updated
 * \param[in] value, the new value
 *
 * When the IOC starts, we will send the soft limits to the controller.
 * When a soft limit is changed, and update is send them to the controller.
 */
asynStatus EthercatMCAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  unsigned indexGroup5000 = 0x5000;
#ifdef motorRecResolutionString
  if (function == pC_->motorRecResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorRecResolution_=%g drvlocal.cfgStepSize=%g\n",
              modulName, axisNo_, value, drvlocal.cfgStepSize);
    if (!drvlocal.cfgStepSize) {
      /* no default from st.cmd. Note: Step size is positiv */
      drvlocal.stepSize = fabs(value);
    }
#endif
  }

  if (function == pC_->motorMoveRel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorMoveRel_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorMoveAbs_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorMoveAbs_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorMoveVel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorMoveVel_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorHome__)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorStop_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorStop_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorVelocity_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorVelocity_=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorVelBase_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorVelBase_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorAccel_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorAccel_)=%g\n", modulName, axisNo_, value);
#if 0
  } else if (function == pC_->motorPosition_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorPosition_=%g\n", axisNo_, value);
  } else if (function == pC_->motorEncoderPosition_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(%d motorEncoderPosition_=%g\n", axisNo_, value);
#endif
  } else if (function == pC_->motorDeferMoves_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motmotorDeferMoves_=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorMoveToHome_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motmotorMoveToHome_=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorResolution_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorResolution_=%g\n",  modulName, axisNo_, value);
  } else if (function == pC_->motorEncoderRatio_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorEncoderRatio_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorPGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorPGain_oveRel_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorIGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorIGain_oveRel_)=%g\n", modulName, axisNo_, value);
  } else if (function == pC_->motorDGain_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoublmotor(%d motorDGain_oveRel_)=%g\n", modulName, axisNo_, value);
    /* Limits handled above */

#ifdef motorPowerOnDelayString
  } else if (function == pC_->motorPowerOnDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorPowerOnDelay_)=%g\n", modulName, axisNo_, value);
#endif
#ifdef motorPowerOffDelayString
  } else if (function == pC_->motorPowerOffDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorPowerOffDelay_=%g\n", modulName, axisNo_, value);
#endif
#ifdef motorPowerOffFractionString
  } else if (function == pC_->motorPowerOffFraction_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motomotorPowerOffFraction_=%g\n", modulName, axisNo_, value);
#endif
#ifdef motorPostMoveDelayString
  } else if (function == pC_->motorPostMoveDelay_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorPostMoveDelay_=%g\n", modulName, axisNo_, value);
#endif
  } else if (function == pC_->motorStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorStatus_)=%g\n", modulName, axisNo_, value);
#ifdef motorRecOffsetString
  } else if (function == pC_->motorRecOffset_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorRecOffset_)=%g\n", modulName, axisNo_, value);
#endif
#ifdef EthercatMCHVELFRMString
  } else if (function == pC_->EthercatMCHVELfrm_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d HVELfrm_)=%g\n", modulName, axisNo_, value);
#endif
#ifdef EthercatMCHomPosString
  } else if (function == pC_->EthercatMCHomPos_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d HomPos_)=%f\n", modulName, axisNo_, value);
#endif
#ifdef EthercatMCCHLMString
  } else if (function == pC_->EthercatMCCHLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCHLM_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xE, value);
    readBackSoftLimits();
    return status;
#endif
#ifdef EthercatMCCLLMString
  } else if (function == pC_->EthercatMCCLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCLLM_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xD, value);
    readBackSoftLimits();
    return status;
#endif
  } else if (function == pC_->EthercatMCCFGVELO_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCFGVELO_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x9, value);
    return status;
  } else if (function == pC_->EthercatMCCFGVMAX_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCFGVMAX_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x27, value);
    return status;
  } else if (function == pC_->EthercatMCCFGJVEL_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCFGJVEL_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x8, value);
    return status;
  } else if (function == pC_->EthercatMCCFGJAR_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCCFGJAR_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(0x4000, 0x101, value);
    return status;
  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

asynStatus EthercatMCAxis::setStringParamDbgStrToMcu(const char *value)
{
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setStringParamDbgStrToMcu(%d)=%s\n",
              modulName, axisNo_, value);
    const char * const Main_this_str = "Main.this.";
    const char * const Sim_this_str = "Sim.this.";
#if 0
    unsigned adsport;
    unsigned indexOffset;
    int      ivalue;
    double   fvalue;
    int nvals = 0;
    int retryCount = 1;
#endif

    /* Check the string. E.g. Main.this. and Sim.this. are passed
       as Main.M1 or Sim.M1
       ADR commands are handled below */
    if (!strncmp(value, Main_this_str, strlen(Main_this_str))) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s",
              drvlocal.adsport_str, axisNo_, value + strlen(Main_this_str));
      return writeReadACK();
    }
    /* caput IOC:m1-DbgStrToMCU Sim.this.log=M1.log */
    if (!strncmp(value, Sim_this_str, strlen(Sim_this_str))) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "Sim.M%d.%s",
              axisNo_, value + strlen(Sim_this_str));
      return writeReadACK();
    }
#if 0
    nvals = sscanf(value, "Sim.M%u.", &ivalue);
    if (nvals == 1) {
      snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", value);
      return writeReadACK();
    }
    /* ADR commands integer
     *  # in  target position monitoring
     *  setADRinteger 501 0x4000 0x15 1
     */
    nvals = sscanf(value, "setADRinteger %u %x %x %d",
                   &indexGroup, &indexOffset, &ivalue);
    if (nvals == 3) {
      return setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                     ivalue, retryCount);
    }
    /* ADR commands floating point
     *  # Target position monitoring window, mm
     *  setADRdouble  501 0x4000 0x6 0.1 */
    nvals = sscanf(value, "setADRdouble %u %x %x %lf",
                   &indexGroup, &indexOffset, &fvalue);

    if (nvals == 3) {
      return setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                     fvalue, retryCount);
    }
#endif
    /* If we come here, the command was not understood */
    return asynError;
}

  asynStatus EthercatMCAxis::setStringParam(int function, const char *value)
{
  if (function == pC_->EthercatMCDbgStrToMcu_) {
    return setStringParamDbgStrToMcu(value);
  } else {
    /* Call base class method */
    return asynMotorAxis::setStringParam(function, value);
  }
}