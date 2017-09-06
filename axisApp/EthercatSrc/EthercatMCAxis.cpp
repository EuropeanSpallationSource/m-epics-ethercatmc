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

#include "EthercatMC.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

/* temporally definition */
#ifndef ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED
#define ERROR_MAIN_ENC_SET_SCALE_FAIL_DRV_ENABLED 0x2001C
#endif

#ifndef ERROR_CONFIG_ERROR
#define ERROR_CONFIG_ERROR 0x30000
#endif

#define NCOMMANDHOME 10

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
  : asynAxisAxis(pC, axisNo),
    pC_(pC)
{
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.old_eeAxisError = eeAxisErrorIOCcomError;
  drvlocal.axisFlags = axisFlags;

  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";
    const char * const cfgfile_str = "cfgFile=";
    const char * const cfgDebug_str = "getDebugText=";

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
        setIntegerParam(pC->motorStatusHasEncoder_, 1);
      }  else if (!strncmp(pThisOption, cfgfile_str, strlen(cfgfile_str))) {
        pThisOption += strlen(cfgfile_str);
        drvlocal.cfgfileStr = strdup(pThisOption);
      } else if (!strncmp(pThisOption, cfgDebug_str, strlen(cfgDebug_str))) {
        pThisOption += strlen(cfgDebug_str);
        drvlocal.cfgDebug_str = strdup(pThisOption);
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
asynStatus EthercatMCAxis::handleDisconnect()
{
  asynStatus status = asynSuccess;
  if (!drvlocal.dirty.oldStatusDisconnected) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s Communication error(%d)\n", modulName, axisNo_);
  }
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.MCU_nErrorId = 0;
  setIntegerParam(pC_->motorStatusCommsError_, 1);
  callParamCallbacksUpdateError();
  return status;
}


void EthercatMCAxis::readBackHighSoftLimit(void)
{
  asynStatus status;
  int iValue = 0;
  double fValue = 0.0;
  /* Soft limits High Enable */
  iValue = 0; fValue = 0.0;
  status = getSAFValuesFromAxisPrint(0x5000, 0xC, &iValue,
                                     0x5000, 0xE, &fValue);
  if (status != asynSuccess) return;
  setIntegerParam(pC_->motorFlagsHighLimitRO_, iValue);
  setDoubleParam(pC_->motorHighLimitRO_, fValue);
  /* EthercatMCDHLMXX are info(asyn:READBACK,"1"),
     so we must use pC_->setXXX here */
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCDHLM_En_, iValue);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCDHLM_, fValue);
}


void EthercatMCAxis::readBackLowSoftLimit(void)
{
  asynStatus status;
  int iValue = 0;
  double fValue = 0.0;
  /* Soft limits Low Enable */
  status = getSAFValuesFromAxisPrint(0x5000, 0xB, &iValue,
                                     0x5000, 0xD, &fValue);
  if (status != asynSuccess) return;
  setDoubleParam(pC_->motorLowLimitRO_, fValue);
  setIntegerParam(pC_->motorFlagsLowLimitRO_, iValue);
  /* EthercatMCDHLMXX are info(asyn:READBACK,"1"),
     so we must use pC_->setXXX(axisNo_..)  here */
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCDLLM_En_, iValue);
  pC_->setDoubleParam(axisNo_, pC_->EthercatMCDLLM_, fValue);
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
void EthercatMCAxis::readBackConfig(void)
{
  asynStatus status;
  int iValue;
  double fValue;
  /* (Micro) steps per revolution */
  status = getSAFValueFromAxisPrint(0x5000, 0x24, &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalSREV_RB_, fValue);

  /* EGU per revolution */
  status = getSAFValueFromAxisPrint(0x5000, 0x23, &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalUREV_RB_, fValue);

  /* Reference Velocity */
  status = getSAFValueFromAxisPrint(0x7000, 0x101, &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalNUM_RB_, fValue);
  /* Motor DIRection */
  status = getSAFValueFromAxisPrint(0x7000, 0x6, &iValue);
  if (status == asynSuccess) setIntegerParam(pC_->EthercatMCScalMDIR_RB_, iValue);
  /* Encoder DIRection */
  status = getSAFValueFromAxisPrint(0x5000, 0x8, &iValue);
  if (status == asynSuccess) setIntegerParam(pC_->EthercatMCScalEDIR_RB_, iValue);

  /* In target position monitor window */
  status = getSAFValueFromAxisPrint(0x4000, 0x16, &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalRDBD_RB_,
                                            fValue);
  /* In target position monitor time */
  status = getSAFValueFromAxisPrint(0x4000, 0x17, &fValue);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCScalRDBD_Tim_RB_,
                                            fValue);

  /* In target position monitor enabled */
  status = getSAFValueFromAxisPrint(0x4000, 0x15, &iValue);
  if (status == asynSuccess) setIntegerParam(pC_->EthercatMCScalRDBD_En_RB_,
                                             iValue);
  readBackHighSoftLimit();
  readBackLowSoftLimit();
}


/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
 * \param[in] AsynStatus status
 *
 * Sets the dirty bits
 */
asynStatus EthercatMCAxis::initialUpdate(void)
{
  asynStatus status = asynSuccess;

  /*  Check for Axis ID */
  int axisID = getMotionAxisID();
  if (axisID != axisNo_) {
    updateMsgTxtFromDriver("ConfigError AxisID");
    return asynError;
  }
  getFeatures();
  status = readConfigFile();
  if (status) return status;

  if ((status == asynSuccess) &&
      (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS)) {
    /* Enable the amplifier when the axis is created,
       but wait until we have a connection to the controller.
       After we lost the connection, Re-enable the amplifier
       See AMPLIFIER_ON_FLAG */
    status = enableAmplifier(1);
  }
  if (status == asynSuccess) readBackConfig();

  if (!status) drvlocal.dirty.initialUpdate = 0;
  return status;
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynAxisAxis::report()
 */
void EthercatMCAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
  }

  // Call the base class method
  asynAxisAxis::report(fp, level);
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
  if (acceleration_time > 0.0001) {
    double acc_in_seconds = maxVelocity / acceleration_time;
    double acc_in_EGU_sec2 = maxVelocity / acc_in_seconds;
    if (acc_in_EGU_sec2  < 0) acc_in_EGU_sec2 = 0 - acc_in_EGU_sec2 ;
    status = setValuesOnAxis("fAcceleration", acc_in_EGU_sec2,
                             "fDeceleration", acc_in_EGU_sec2);
    if (status) return status;
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s sendVelocityAndAccelExecute(%d) maxVelocity=%g acceleration_time=%g\n",
               modulName, axisNo_, maxVelocity, acceleration_time);
  }
  status = setValueOnAxis("fVelocity", maxVelocity);
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
  drvlocal.waitNumPollsBeforeReady += 2;
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
  int nCommand = relative ? 2 : 3;
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) drvlocal.nCommand = nCommand;
  if (status == asynSuccess) status = setValueOnAxis("fPosition", position);
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

  int procHom;
  double posHom;
  double velToHom;
  double velFrmHom;
  double accHom;
  double decHom;

  if (status == asynSuccess) status = pC_->getIntegerParam(axisNo_,
                                                           pC_->EthercatMCProcHom_,
                                                           &procHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCPosHom_,
                                                          &posHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCVelToHom_,
                                                          &velToHom);
  if (status == asynSuccess) setDoubleParam(pC_->EthercatMCVel_RB_, velToHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCVelFrmHom_,
                                                          &velFrmHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCAccHom_,
                                                          &accHom);
  if (status == asynSuccess) status = pC_->getDoubleParam(axisNo_,
                                                          pC_->EthercatMCDecHom_,
                                                          &decHom);

  /* The controller will do the home search, and change its internal
     raw value to what we specified in fPosition. */
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if ((drvlocal.axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) &&
      (status == asynSuccess)) status = enableAmplifier(1);
  if (status == asynSuccess) status = setValueOnAxis("fHomePosition", posHom);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", nCommand );
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", procHom);

  if (status == asynSuccess) status = setSAFValueOnAxis(0x4000, 0x6,
                                                        velToHom);
  if (status == asynSuccess) status = setSAFValueOnAxis(0x4000, 0x7,
                                                        velFrmHom);
  if (status == asynSuccess)  status = setValuesOnAxis("fAcceleration", accHom,
                                                       "fDeceleration", decHom);

  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
  if (status == asynSuccess) drvlocal.nCommand = nCommand;
  drvlocal.waitNumPollsBeforeReady += 2;
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
  if (status == asynSuccess) setValueOnAxis("nCommand", 1);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}



/**
 * See asynMotorAxis::setPosition
 */
asynStatus EthercatMCAxis::setPosition(double value)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s setPosition(%d position=%g\n",
             modulName, axisNo_, value);
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

/** Enable the amplifier on an axis
 *
 */
asynStatus EthercatMCAxis::enableAmplifier(int on)
{
  asynStatus status = asynSuccess;
  unsigned counter = 10;
  bool moving;
  int ret;
  on = on ? 1 : 0; /* either 0 or 1 */
  status = getValueFromAxis("bEnabled", &ret);
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
    sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s?;Main.M%d.%s?",
            drvlocal.adsport, axisNo_, "bBusy", axisNo_, "bEnabled");
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s out=%s in=%s status=%s (%d)\n",
               modulName, pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);
    if (status) return status;
    if (!strcmp("0;1", pC_->inString_)) {
      /* bBusy == 0; bEnabled == 1 */
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
  drvlocal.nCommand = 0;
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s stopAxisInternal(%d) (%s)\n",   modulName, axisNo_, function_name);
  status = setValueOnAxisVerify("bExecute", "bExecute", 0, 1);
  if (status) drvlocal.mustStop = 1;
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
  if (EPICS_nErrorId) {
    /* Error from MCU */
    drvlocal.eeAxisError = eeAxisErrorMCUError;
  } else if (drvlocal.dirty.sErrorMessage) {
    /* print error below */
    drvlocal.eeAxisError = eeAxisErrorIOCcomError;
  } else if (drvlocal.dirty.readConfigFile) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    drvlocal.eeAxisError = eeAxisErrorIOCcfgError;
          updateMsgTxtFromDriver("ConfigError Config File");
  } else if (drvlocal.dirty.initialUpdate) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    updateMsgTxtFromDriver("ConfigError");
  } else if (drvlocal.dirty.nMotionAxisID != axisNo_) {
    EPICS_nErrorId = ERROR_CONFIG_ERROR;
    updateMsgTxtFromDriver("ConfigError: AxisID");
  } else if (drvlocal.cmdErrorMessage[0]) {
    drvlocal.eeAxisError = eeAxisErrorCmdError;
  }
  if (drvlocal.eeAxisError != drvlocal.old_eeAxisError ||
      drvlocal.old_EPICS_nErrorId != EPICS_nErrorId) {

    if (!drvlocal.cfgDebug_str) {
      if (!EPICS_nErrorId)
        updateMsgTxtFromDriver(NULL);

      switch (drvlocal.eeAxisError) {
        case eeAxisErrorNoError:
          updateMsgTxtFromDriver(NULL);
          break;
        case eeAxisErrorIOCcomError:
          updateMsgTxtFromDriver("CommunicationError");
          break;
        case eeAxisErrorCmdError:
          updateMsgTxtFromDriver(drvlocal.cmdErrorMessage);
          break;
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
              "%s poll(%d) callParamCallbacksUpdateError eeAxisError=%d old=%d ErrID=0x%x old=0x%x\n",
               modulName, axisNo_, drvlocal.eeAxisError, drvlocal.old_eeAxisError,
              EPICS_nErrorId, drvlocal.old_EPICS_nErrorId);

    drvlocal.old_eeAxisError = drvlocal.eeAxisError;
    drvlocal.old_EPICS_nErrorId = EPICS_nErrorId;
  }

  callParamCallbacks();
}


asynStatus EthercatMCAxis::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
{
  asynStatus comStatus;

  int motor_axis_no = 0;
  int nvals = 0;
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
  if (drvlocal.supported.stAxisStatus_V2 || drvlocal.dirty.stAxisStatus_Vxx) {
    /* V2 is supported, use it. Or. unkown: try it as well */
    sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.stAxisStatusV2?", drvlocal.adsport, axisNo_);
    comStatus = pC_->writeReadOnErrorDisconnect();
    nvals = sscanf(pC_->inString_,
                   "Main.M%d.stAxisStatusV2="
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
    if (nvals == 27) {
      if (drvlocal.dirty.stAxisStatus_Vxx) {
        drvlocal.supported.stAxisStatus_V2 = 1;
        setIntegerParam(pC_->motorStatusHasEncoder_, 1);
      }
      pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy || !pst_axis_status->atTarget;
    }
  }
  if (!drvlocal.supported.stAxisStatus_V2) {
    /* Read the complete Axis status */
    sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.stAxisStatus?", drvlocal.adsport, axisNo_);
    comStatus = pC_->writeReadOnErrorDisconnect();
    if (comStatus) return comStatus;
    nvals = sscanf(pC_->inString_,
                   "Main.M%d.stAxisStatus="
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

    if (nvals != 24) {
      drvlocal.supported.stAxisStatus_V1 = 0;
      goto pollAllWrongnvals;
    }
    drvlocal.supported.stAxisStatus_V1 = 1;

    /* V1 new style: mvnNRdyNex follows bBusy */
    if (drvlocal.supported.bSIM || drvlocal.supported.bECMC)
      drvlocal.supported.bBusyNewStyle = 1;

    pst_axis_status->mvnNRdyNex = pst_axis_status->bBusy && pst_axis_status->bEnabled;
    if (!drvlocal.supported.bBusyNewStyle) {
      /* "V1 old style":done when bEcecute is 0 */
      pst_axis_status->mvnNRdyNex &= pst_axis_status->bExecute;
    }
  } /* End of V1 */
  /* From here on, either V1 or V2 is supported */
  if (drvlocal.dirty.stAxisStatus_Vxx) {
    if (drvlocal.supported.stAxisStatus_V2)
      drvlocal.supported.statusVer = 2;
    else if (drvlocal.supported.stAxisStatus_V1 && !drvlocal.supported.bBusyNewStyle)
      drvlocal.supported.statusVer = 0;
    else if (drvlocal.supported.stAxisStatus_V1 && drvlocal.supported.bBusyNewStyle)
      drvlocal.supported.statusVer = 1;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s pollAll(%d) nvals=%d V1=%d V2=%d sim=%d ecmc=%d bBusyNewStyle=%d Ver=%d fActPosition=%f\n",
              modulName, axisNo_, nvals,
              drvlocal.supported.stAxisStatus_V1,
              drvlocal.supported.stAxisStatus_V2,
              drvlocal.supported.bSIM,
              drvlocal.supported.bECMC,
              drvlocal.supported.bBusyNewStyle,
              drvlocal.supported.statusVer,
              pst_axis_status->fActPosition);
    setIntegerParam(pC_->motorFlagsHomeOnLs_, 1);
    setIntegerParam(pC_->motorFlagsStopOnProblem_, 0);
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
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s pollAll(%d) nvals=%d in=%s\n",
            modulName, axisNo_, nvals, pC_->inString_);
  return asynError;
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

  memset(&st_axis_status, 0, sizeof(st_axis_status));
  /* Try to read to see if the connection is up */
  if (drvlocal.dirty.nMotionAxisID < 0) {
    int ret;
    comStatus = getValueFromAxis("nMotionAxisID", &ret);
    if (comStatus) goto skip;
    if (ret >= 0) drvlocal.dirty.nMotionAxisID = ret;
  }

  /* Stop if the previous stop had been lost */
  if (drvlocal.mustStop) {
    comStatus = stopAxisInternal(__FUNCTION__, 0);
    if (comStatus) goto skip;
  }
  if (drvlocal.dirty.initialUpdate) {
    comStatus = initialUpdate();
    if (comStatus) {
      callParamCallbacksUpdateError();
      return asynError;
    }
    if (drvlocal.dirty.oldStatusDisconnected) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s connected(%d)\n",  modulName, axisNo_);
      drvlocal.dirty.oldStatusDisconnected = 0;
    }
  }

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
    sprintf(pC_->outString_, "%s", drvlocal.cfgDebug_str);
    comStatus = pC_->writeReadOnErrorDisconnect();
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
  /*
   * Special : "-En" reflects the hardware status. So if the value changes
   * in the controller, the PV will reflect it.
   * See info(asyn:READBACK,"1") in EthercatMC.template
  */
  pC_->setIntegerParam(axisNo_, pC_->EthercatMCEn_, st_axis_status.bEnabled);

  setDoubleParam(pC_->EthercatMCVelAct_, st_axis_status.fActVelocity);
  setDoubleParam(pC_->EthercatMCAcc_RB_, st_axis_status.fAcceleration);
  setDoubleParam(pC_->EthercatMCDec_RB_, st_axis_status.fDecceleration);

  if (drvlocal.waitNumPollsBeforeReady) {
    *moving = true;
  } else {
    *moving = st_axis_status.mvnNRdyNex ? true : false;
    if (!st_axis_status.mvnNRdyNex) drvlocal.nCommand = 0;
  }

  if (drvlocal.nCommand != NCOMMANDHOME) {
    setDoubleParam(pC_->motorPosition_, st_axis_status.fActPosition);
    drvlocal.old_st_axis_status.fActPosition = st_axis_status.fActPosition;
    setDoubleParam(pC_->EthercatMCVel_RB_, st_axis_status.fVelocity);
  }

  if (drvlocal.externalEncoderStr) {
    comStatus = getValueFromController(drvlocal.externalEncoderStr,
                                       &st_axis_status.positionRaw);
    if (!comStatus) setDoubleParam(pC_->motorEncoderPosition_,
                                   st_axis_status.positionRaw);
  } else if (drvlocal.supported.stAxisStatus_V2) {
    setDoubleParam(pC_->motorEncoderPosition_, st_axis_status.positionRaw);
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
  } else {
    if (drvlocal.old_st_axis_status.mvnNRdyNex != st_axis_status.mvnNRdyNex ||
        drvlocal.old_st_axis_status.bBusy      != st_axis_status.bBusy ||
        drvlocal.old_st_axis_status.bEnabled   != st_axis_status.bEnabled ||
        drvlocal.old_st_axis_status.bExecute   != st_axis_status.bExecute ||
        drvlocal.old_st_axis_status.atTarget   != st_axis_status.atTarget) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s poll(%d) mvnNRdyNexAt=%d Ver=%d bBusy=%d bExecute=%d bEnabled=%d atTarget=%d ENC=%g fActPosition=%g\n",
                 modulName, axisNo_, st_axis_status.mvnNRdyNex,
                drvlocal.supported.statusVer,
                st_axis_status.bBusy, st_axis_status.bExecute,
                st_axis_status.bEnabled, st_axis_status.atTarget,
                st_axis_status.positionRaw, st_axis_status.fActPosition);
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
  }
  memcpy(&drvlocal.old_st_axis_status, &st_axis_status,
         sizeof(drvlocal.old_st_axis_status));
  return asynSuccess;

  skip:
  handleDisconnect();
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
  if (function == pC_->EthercatMCEn_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d EthercatMCEn_)=%d\n",
              modulName, axisNo_, value);
    /* Set it in the parameter lib; the poller may update it later */
    pC_->setIntegerParam(axisNo_, pC_->EthercatMCEn_, value);
    status = enableAmplifier(value);
    return status;

  } else if (function == pC_->motorUpdateStatus_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d motorUpdateStatus_)=%d\n", modulName, axisNo_, value);
    initialUpdate();
#ifdef motorRecDirectionString
  } else if (function == pC_->motorRecDirection_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d motorRecDirection_)=%d\n",
              modulName, axisNo_, value);
#endif
#ifdef EthercatMCProcHomString
  } else if (function == pC_->EthercatMCProcHom_) {
    /* If value != 0 the axis can be homed. Show warning if it isn't homed */
    setIntegerParam(pC_->motorFlagsShowNotHomed_, value);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d ProcHom_)=%d\n",  modulName, axisNo_, value);
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
#ifdef EthercatMCDHLM_EnString
  } else if (function == pC_->EthercatMCDHLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d EthercatMCDHLM_En)=%d\n",
              modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xC, value);
    readBackHighSoftLimit();
    return status;
#endif
#ifdef EthercatMCDLLM_EnString
  } else if (function == pC_->EthercatMCDLLM_En_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setIntegerParam(%d EthercatMCDLLM_En)=%d\n",
              modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xB, value);
    readBackLowSoftLimit();
    return status;
#endif
  }

  //Call base class method
  status = asynAxisAxis::setIntegerParam(function, value);
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
              "%s setDoubleParam(%d motorRecResolution_=%g\n",
              modulName, axisNo_, value);
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

#ifdef motorPowerAutoOnOffString
  } else if (function == pC_->motorPowerAutoOnOff_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d motorPowerAutoOnOff_%g\n", modulName, axisNo_, value);
#endif
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
#ifdef EthercatMCPosHomString
  } else if (function == pC_->EthercatMCPosHom_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d PosHom_)=%f\n", modulName, axisNo_, value);
#endif
#ifdef EthercatMCDHLMString
  } else if (function == pC_->EthercatMCDHLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCDHLM_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xE, value);
    readBackHighSoftLimit();
    return status;
#endif
#ifdef EthercatMCDLLMString
  } else if (function == pC_->EthercatMCDLLM_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setDoubleParam(%d EthercatMCDLLM_)=%f\n", modulName, axisNo_, value);
    status = setSAFValueOnAxis(indexGroup5000, 0xD, value);
    readBackLowSoftLimit();
    return status;
#endif
  }
  // Call the base class method
  status = asynAxisAxis::setDoubleParam(function, value);
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
      sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s",
              drvlocal.adsport, axisNo_, value + strlen(Main_this_str));
      return writeReadACK();
    }
    /* caput IOC:m1-DbgStrToMCU Sim.this.log=M1.log */
    if (!strncmp(value, Sim_this_str, strlen(Sim_this_str))) {
      sprintf(pC_->outString_, "Sim.M%d.%s",
              axisNo_, value + strlen(Sim_this_str));
      return writeReadACK();
    }
#if 0
    nvals = sscanf(value, "Sim.M%u.", &ivalue);
    if (nvals == 1) {
      sprintf(pC_->outString_, "%s", value);
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
    return asynAxisAxis::setStringParam(function, value);
  }
}
