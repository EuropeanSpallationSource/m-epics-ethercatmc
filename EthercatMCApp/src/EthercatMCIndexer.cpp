/*
  FILENAME... EthercatMCIndexer.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>

#include "EthercatMCController.h"
#include "EthercatMCIndexerAxis.h"

#include <epicsThread.h>

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

extern "C" {
  const char *plcUnitTxtFromUnitCode(unsigned unitCode)
  {
    const static char *const unitTxts[] = {
      " ",
      "V",
      "A",
      "W",
      "m",
      "gr",
      "Hz",
      "T",
      "K",
      "C",
      "F",
      "bar",
      "degree",
      "Ohm",
      "m/sec",
      "m2/sec",
      "m3/sec",
      "s",
      "counts",
      "bar/sec",
      "bar/sec2",
      "F",
      "H" };
    if (unitCode < sizeof(unitTxts)/sizeof(unitTxts[0]))
      return unitTxts[unitCode];

    return "??";
  }
  const char *plcUnitPrefixTxt(int prefixCode)
  {
    if (prefixCode >= 0) {
      switch (prefixCode) {
      case 24:   return "Y";
      case 21:   return "Z";
      case 18:   return "E";
      case 15:   return "P";
      case 12:   return "T";
      case 9:    return "G";
      case 6:    return "M";
      case 3:    return "k";
      case 2:    return "h";
      case 1:    return "da";
      case 0:    return "";
      default:
        return "?";
      }
    } else {
      switch (-prefixCode) {
      case 1:   return "d";
      case 2:   return "c";
      case 3:   return "m";
      case 6:   return "u";
      case 9:   return "n";
      case 12:  return "p";
      case 15:  return "f";
      case 18:  return "a";
      case 21:  return "z";
      case 24:  return "y";
      default:
        return "?";
      }
    }
  }
};

static const double fABSMIN = -3.0e+38;
static const double fABSMAX =  3.0e+38;

asynStatus EthercatMCController::getPlcMemoryUint(unsigned indexOffset,
                                                  unsigned *value,
                                                  size_t lenInPlc)
{
  asynStatus status;

  memset(value, 0, lenInPlc);
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    status = getPlcMemoryViaADS(indexOffset, &raw, lenInPlc);
    *value = netToUint(&raw, lenInPlc);
    return status;
  }
  return asynError;
}

asynStatus EthercatMCController::getPlcMemoryString(unsigned indexOffset,
                                                    char *value,
                                                    size_t len)
{
  memset(value, 0, len);
  return getPlcMemoryViaADS(indexOffset, value, len);
}

asynStatus EthercatMCController::setPlcMemoryInteger(unsigned indexOffset,
                                                     int value,
                                                     size_t lenInPlc)
{
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    uintToNet(value, &raw, lenInPlc);
    return setPlcMemoryViaADS(indexOffset, raw, lenInPlc);
  } else {
    return asynError;
  }
}


asynStatus EthercatMCController::getPlcMemoryDouble(unsigned indexOffset,
                                                    double *value,
                                                    size_t lenInPlc)
{
  asynStatus status;

  memset(value, 0, lenInPlc);
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    status = getPlcMemoryViaADS(indexOffset, &raw, lenInPlc);
    *value = netToDouble(&raw, lenInPlc);
    return status;
  }
  return asynError;
}

asynStatus EthercatMCController::setPlcMemoryDouble(unsigned indexOffset,
                                                    double value,
                                                    size_t lenInPlc)
{
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    doubleToNet(value, &raw, lenInPlc);
    return setPlcMemoryViaADS(indexOffset, &raw, lenInPlc);
  }
  return asynError;
}


asynStatus EthercatMCController::readDeviceIndexer(unsigned devNum,
                                                   unsigned infoType)
{
  asynStatus status;
  unsigned value = (devNum + (infoType << 8));
  unsigned valueAcked = 0x8000 + value;
  unsigned counter = 0;
  if ((devNum > 0xFF) || (infoType > 0xFF)) {
    status = asynDisabled;
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%sreadDeviceIndexer devNum=%u infoType=%u status=%s (%d)\n",
              modNamEMC,devNum, infoType,
              EthercatMCstrStatus(status), (int)status);
    return status;
  }

  /* https://forge.frm2.tum.de/public/doc/plc/master/singlehtml/
     The ACK bit on bit 15 must be set when we read back.
     devNum and infoType must match our request as well,
     otherwise there is a collision.
  */
  status = setPlcMemoryInteger(ctrlLocal.indexerOffset, value, 2);
  if (status) {
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%sreadDeviceIndexer status=%s (%d)\n",
              modNamEMC,
              EthercatMCstrStatus(status), (int)status);
    return status;
  }
  while (counter < 5) {
    status = getPlcMemoryUint(ctrlLocal.indexerOffset, &value, 2);
    if (status) {
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%sreadDeviceIndexer status=%s (%d)\n",
                modNamEMC,
                EthercatMCstrStatus(status), (int)status);
      return status;
    }
    if (value == valueAcked) return asynSuccess;
    counter++;
    epicsThreadSleep(.1 * (counter<<1));
  }
  status = asynDisabled;
  asynPrint(pasynUserController_,
            ASYN_TRACE_INFO,
            "%sreadDeviceIndexer devNum=0x%x infoType=0x%x counter=%u value=0x%x status=%s (%d)\n",
            modNamEMC, devNum, infoType, counter, value,
            EthercatMCstrStatus(status), (int)status);
  return status;

}

asynStatus EthercatMCController::indexerParamWaitNotBusy(unsigned indexOffset)
{
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned   cmdSubParamIndex = 0;
  unsigned   counter = 0;

  while (counter < 5) {
    status = getPlcMemoryUint(indexOffset, &cmdSubParamIndex, 2);
    asynPrint(pasynUserController_,
              status ? traceMask | ASYN_TRACE_INFO : traceMask,
              "%scmdSubParamIndex=0x%04x status=%s (%d)\n",
              modNamEMC, cmdSubParamIndex,
              EthercatMCstrStatus(status), (int)status);
    if (status) return status;
    switch (cmdSubParamIndex & PARAM_IF_CMD_MASK) {
    case PARAM_IF_CMD_INVALID:
    case PARAM_IF_CMD_DONE:
    case PARAM_IF_CMD_ERR_NO_IDX:
    case PARAM_IF_CMD_READONLY:
    case PARAM_IF_CMD_RETRY_LATER:
      return asynSuccess;
    case PARAM_IF_CMD_BUSY:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sBUSY\n",
                modNamEMC);
      return asynDisabled;
    default:
      ; /* Read, write continue looping */
    }
    counter++;
    epicsThreadSleep(.1 * (counter<<1));
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sindexOffset=%u cmdSubParamIndex=0x%04x counter=%d\n",
            modNamEMC, indexOffset, cmdSubParamIndex, counter);
  return asynDisabled;
}


asynStatus EthercatMCController::indexerParamRead(int axisNo,
                                                  unsigned paramIfOffset,
                                                  unsigned paramIndex,
                                                  unsigned lenInPlcPara,
                                                  double   *value)
{
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned cmd      = PARAM_IF_CMD_DOREAD + paramIndex;
  unsigned cmdAcked = PARAM_IF_CMD_DONE   + paramIndex;
  unsigned lenInPlcCmd = 2;
  unsigned counter = 0;

  if (paramIndex > 0xFF) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s paramIndex=%d\n",
              modNamEMC, paramIndex);
    return asynDisabled;
  }
  if (lenInPlcPara <= 8) {
    status = indexerParamWaitNotBusy(paramIfOffset);
    if (status) return status;

    /*
      The parameter interface has this layout:
      0 CmdParamReasonIdx
      2 ParamValue
    */

    status = setPlcMemoryInteger(paramIfOffset, cmd, lenInPlcCmd);
    asynPrint(pasynUserController_, traceMask | (status ? ASYN_TRACE_ERROR : 0),
	      "%sstatus=%s (%d)\n",
	      modNamEMC,
	      EthercatMCstrStatus(status), (int)status);
    if (status) return status;
    while (counter < 5) {
      unsigned cmdSubParamIndex = 0;
      double fValue;
      struct {
	uint8_t   paramCtrl[2];
	uint8_t   paramValue[8];
      } paramIf;
      status = getPlcMemoryViaADS(paramIfOffset,
				  &paramIf,
                                  sizeof(paramIf.paramCtrl) + lenInPlcPara);
      if (status) {
	asynPrint(pasynUserController_, traceMask | ASYN_TRACE_ERROR,
		  "%sstatus=%s (%d)\n",
		  modNamEMC,
		  EthercatMCstrStatus(status), (int)status);
	return status;
      }
      cmdSubParamIndex = netToUint(&paramIf.paramCtrl,
                                   sizeof(paramIf.paramCtrl));
      if (paramIndex < 30) {
        /* parameters below 30 are unsigned integers in the PLC
           Read them as integers from PLC, and parse into a double */
	fValue           = (double)netToUint(&paramIf.paramValue,
                                             lenInPlcPara);
      } else {
	fValue           = netToDouble(&paramIf.paramValue,lenInPlcPara);
      }
      if (cmdSubParamIndex == cmdAcked) {
	/* This is good, return */
	*value = fValue;
	return asynSuccess;
      }

      switch (cmdSubParamIndex & PARAM_IF_CMD_MASK) {
      case PARAM_IF_CMD_INVALID:
        status = asynDisabled;
      case PARAM_IF_CMD_DOREAD:
        status = asynDisabled;
      case PARAM_IF_CMD_DOWRITE:
      case PARAM_IF_CMD_BUSY:
        break;
      case PARAM_IF_CMD_DONE:
        /* This is an error. (collision ?) */
        status = asynDisabled;
      case PARAM_IF_CMD_ERR_NO_IDX:
        status = asynDisabled;
      case PARAM_IF_CMD_READONLY:
        status = asynDisabled;
      case PARAM_IF_CMD_RETRY_LATER:
        status = asynDisabled;
      }
      if (status && (counter > 1)) {
	asynPrint(pasynUserController_, traceMask | ASYN_TRACE_ERROR,
		  "%s (%d) paramIfOffset=%u paramIndex=%u cmdSubParamIndex=0x%04x counter=%u status=%s (%d)\n",
		  modNamEMC, axisNo, paramIfOffset, paramIndex, cmdSubParamIndex,
		  counter,
		  EthercatMCstrStatus(status), (int)status);
      }
      epicsThreadSleep(.1 * (counter<<1));
      counter++;
    }
    return asynDisabled;
  } /* lenInPlcPara == 4 */
  return asynError;
}

asynStatus EthercatMCController::indexerParamWrite(unsigned paramIfOffset,
                                                   unsigned paramIndex,
                                                   unsigned lenInPlcPara,
                                                   double value)
{
  unsigned traceMask = ASYN_TRACE_INFO;
  asynStatus status;
  unsigned cmd      = PARAM_IF_CMD_DOWRITE + paramIndex;
  unsigned cmdAcked = PARAM_IF_CMD_DONE    + paramIndex;
  size_t lenInPlcCmd = 2;
  unsigned counter = 0;

  if (paramIndex > 0xFF) return asynDisabled;
  status = indexerParamWaitNotBusy(paramIfOffset);
  if (status) return status;

  /*
    The parameter interface has this layout:
    0 CmdParamReasonIdx
    2 ParamValue
  */
  /* Parameters 1..4 are integers, the rest is floating point */
  if (paramIndex <= 4)
    status = setPlcMemoryInteger(paramIfOffset + 2, (int)value, lenInPlcPara);
  else
    status = setPlcMemoryDouble(paramIfOffset + 2, value, lenInPlcPara);
  if (status) traceMask |= ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER;
  asynPrint(pasynUserController_, traceMask,
            "%sparamIfOffset=%u paramIndex=%u value=%f lenInPlcPara=%u status=%s (%d)\n",
            modNamEMC, paramIfOffset, paramIndex, value, lenInPlcPara,
            EthercatMCstrStatus(status), (int)status);
  if (status) return status;

  status = setPlcMemoryInteger(paramIfOffset, cmd, lenInPlcCmd);
  if (status) traceMask |= ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER;
  asynPrint(pasynUserController_, traceMask,
            "%sstatus=%s (%d)\n",
            modNamEMC,
            EthercatMCstrStatus(status), (int)status);
  if (status) return status;
  while (counter < 5) {
    unsigned cmdSubParamIndex = 0;
    status = getPlcMemoryUint(paramIfOffset, &cmdSubParamIndex, 2);
    if (status) traceMask |= ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER;
    asynPrint(pasynUserController_, traceMask,
              "%sparamIndex=%u lenInPlcPara=%u cmdSubParamIndex=0x%04x counter=%u status=%s (%d)\n",
              modNamEMC,
              paramIndex, lenInPlcPara, cmdSubParamIndex, counter,
              EthercatMCstrStatus(status), (int)status);
    if (status) return status;
    /* This is good, return */
    if (cmdSubParamIndex == cmdAcked) return asynSuccess;
    switch (cmdSubParamIndex & PARAM_IF_CMD_MASK) {
    case PARAM_IF_CMD_INVALID:
      status = asynDisabled;
    case PARAM_IF_CMD_DOREAD:
      status = asynDisabled;
    case PARAM_IF_CMD_DOWRITE:
    case PARAM_IF_CMD_BUSY:
      break;
    case PARAM_IF_CMD_DONE:
      /* This is an error. (collision ?) */
      status = asynDisabled;
    case PARAM_IF_CMD_ERR_NO_IDX:
      status = asynDisabled;
    case PARAM_IF_CMD_READONLY:
      status = asynDisabled;
    case PARAM_IF_CMD_RETRY_LATER:
      status = asynDisabled;
    }
    if (status) {
      traceMask |= ASYN_TRACE_INFO;
      asynPrint(pasynUserController_, traceMask,
                "%scmdSubParamIndex=0x%04x counter=%u status=%s (%d)\n",
                modNamEMC, cmdSubParamIndex,
                counter,
                EthercatMCstrStatus(status), (int)status);
      return status;
    }
    epicsThreadSleep(.1 * (counter<<1));
    counter++;
  }
  asynPrint(pasynUserController_,
            traceMask | ASYN_TRACE_INFO,
            "%scounter=%u\n",
            modNamEMC, counter);
  return asynDisabled;
}

void EthercatMCController::parameterFloatReadBack(unsigned axisNo,
                                                  unsigned paramIndex,
                                                  double fValue)
{
  asynMotorAxis *pAxis=getAxis((int)axisNo);
  const static double fullsrev = 200;    /* (default) Full steps/revolution */

  switch(paramIndex) {
  case PARAM_IDX_OPMODE_AUTO_UINT32:
    /* CNEN for EPICS */
    pAxis->setIntegerParam(motorStatusGainSupport_, 1);
    break;
  case PARAM_IDX_MICROSTEPS_FLOAT32:
    pAxis->setDoubleParam(EthercatMCCfgSREV_RB_, fullsrev * fValue);
    break;
  case PARAM_IDX_USR_MIN_FLOAT32:
    setIntegerParam(axisNo,EthercatMCCfgDLLM_En_, fValue > fABSMIN ? 1 : 0);
    setDoubleParam(axisNo, EthercatMCCfgDLLM_,    fValue);
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_ABS_MIN_FLOAT32:
    setDoubleParam(axisNo,  EthercatMCCfgPMIN_RB_, fValue);
    break;
  case PARAM_IDX_ABS_MAX_FLOAT32:
    setDoubleParam(axisNo,  EthercatMCCfgPMAX_RB_, fValue);
    break;
  case PARAM_IDX_USR_MAX_FLOAT32:
    setIntegerParam(axisNo, EthercatMCCfgDHLM_En_, fValue < fABSMAX ? 1 : 0);
    setDoubleParam(axisNo,  EthercatMCCfgDHLM_, fValue);
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_WRN_MIN_FLOAT32:
    break;
  case PARAM_IDX_WRN_MAX_FLOAT32:
    break;
  case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT32:
    pAxis->setDoubleParam(EthercatMCCfgPOSLAG_RB_, fValue);
    setIntegerParam(axisNo, EthercatMCCfgPOSLAG_En_RB_, 1);
    break;
  case PARAM_IDX_HYTERESIS_FLOAT32:
    pAxis->setDoubleParam(EthercatMCCfgSPDB_RB_, fValue);
    pAxis->setDoubleParam(EthercatMCCfgRDBD_RB_, fValue);
    setIntegerParam(axisNo, EthercatMCCfgRDBD_En_RB_, 1);
#ifdef motorRDBDROString
    pAxis->setDoubleParam(motorRDBDRO_, fValue);
#endif
    break;
  case PARAM_IDX_REFSPEED_FLOAT32:
    pAxis->setDoubleParam(EthercatMCVelToHom_, fValue);
    break;
  case PARAM_IDX_SPEED_FLOAT32:
    pAxis->setDoubleParam(EthercatMCCfgVELO_, fValue);
    pAxis->setDoubleParam(EthercatMCVel_RB_, fValue);
#ifdef motorDefVelocityROString
    pAxis->setDoubleParam(motorDefVelocityRO_, fValue);
#endif
    break;
  case PARAM_IDX_ACCEL_FLOAT32:
    pAxis->setDoubleParam(EthercatMCCfgACCS_, fValue);
    pAxis->setDoubleParam(EthercatMCAcc_RB_, fValue);
#ifdef motorDefJogAccROString
    pAxis->setDoubleParam(motorDefJogAccRO_, fValue);
#endif
    break;
  case PARAM_IDX_IDLE_CURRENT_FLOAT32:
    break;
  case PARAM_IDX_MOVE_CURRENT_FLOAT32:
    break;
  case PARAM_IDX_MICROSTEPS_UINT32:
    break;
  case PARAM_IDX_STEPS_PER_UNIT_FLOAT32:
    {
      double urev = fabs(fullsrev / fValue);
      pAxis->setDoubleParam(EthercatMCCfgUREV_RB_, urev);
    }
    break;
  case PARAM_IDX_HOME_POSITION_FLOAT32:
    pAxis->setDoubleParam(EthercatMCHomPos_, fValue);
    break;
  case PARAM_IDX_FUN_REFERENCE:
#ifdef  motorNotHomedProblemString
    pAxis->setIntegerParam(motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
    pAxis->setIntegerParam(EthercatMCHomProc_RB_, 14);
    break;
  case PARAM_IDX_FUN_SET_POSITION:
#ifdef  motorNotHomedProblemString
    pAxis->setIntegerParam(motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
    pAxis->setIntegerParam(EthercatMCHomProc_RB_, 15);
    break;
  }
}

asynStatus
EthercatMCController::indexerReadAxisParameters(EthercatMCIndexerAxis *pAxis,
                                                unsigned devNum,
                                                unsigned iOffset,
                                                unsigned lenInPlcPara)
{
  unsigned axisNo = pAxis->axisNo_;
  unsigned infoType15 = 15;
  asynStatus status;
  unsigned dataIdx;

  status = readDeviceIndexer(devNum, infoType15);
  if (status) {
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%sindexerReadAxisParameters status=%s (%d)\n",
              modNamEMC,
              EthercatMCstrStatus(status), (int)status);
    return status;
  }
  for (dataIdx = 0; dataIdx <= 16; dataIdx++) {
    unsigned parameters;
    int traceMask = 0;
    parameters = -1;

    status = getPlcMemoryUint(ctrlLocal.indexerOffset + (1 + dataIdx) * 2, &parameters, 2);
    if (status) {
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%sindexerReadAxisParameters (%d) status=%s (%d)\n",
                modNamEMC, axisNo,
                EthercatMCstrStatus(status), (int)status);
      return status;
    }
    /* dataIdx == 0 has ACK + infoType/devNum
       dataIdx == 1 has supported parameters 15..0 */
    asynPrint(pasynUserController_, traceMask,
              "%sparameters[%03u..%03u]=0x%04x\n",
              modNamEMC, dataIdx*16 +15,
              dataIdx*16, parameters);
    /* Where is the parameter interface to this device ?
       See indexerDevice5008interface_type;,
       indexerDevice5010interface_type in indexer.c */
    unsigned paramIfOffset;
    switch (lenInPlcPara) {
    case 4:
      paramIfOffset = iOffset + 10;
      break;
    case 8:
      paramIfOffset = iOffset + 22;
      break;
    default:
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%sindexerReadAxisParameters(%d) asynError: lenInPlcPara=%u \n",
                modNamEMC, axisNo, lenInPlcPara);
      return asynError;
    }
    unsigned bitIdx;

    for (bitIdx = 0; bitIdx <= 15; bitIdx++) {
      unsigned paramIndex = dataIdx*16 + bitIdx;
      unsigned bitIsSet = parameters & (1 << bitIdx) ? 1 : 0;
      if (bitIsSet) {
        double fValue = 0.0;
        if (paramIndex < 128) {
          /* paramIndex >= 128 are functions.
             Don't read them.
             tell driver that the function exist */
          status = indexerParamRead(axisNo,
                                    paramIfOffset,
                                    paramIndex,
                                    lenInPlcPara,
                                    &fValue);
          if (status) {
            asynPrint(pasynUserController_,
                      ASYN_TRACE_INFO,
                      "%sindexerReadAxisParameters paramIndex=%u lenInPlcPara=%u status=%s (%d)\n",
                      modNamEMC, paramIndex, lenInPlcPara,
                      EthercatMCstrStatus(status), (int)status);
            return status;
          }
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%u fValue=%f\n",
                    modNamEMC, axisNo, paramIndex, fValue);
        } else {
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdxFunction=%u\n",
                    modNamEMC, axisNo, paramIndex);
        }
        parameterFloatReadBack(axisNo, paramIndex, fValue);
      }
    }
  }
  return asynSuccess;
}

asynStatus
EthercatMCController::newIndexerAxis(EthercatMCIndexerAxis *pAxis,
                                     unsigned devNum,
                                     unsigned iAllFlags,
                                     double   fAbsMin,
                                     double   fAbsMax,
                                     unsigned iOffset)
{
  asynStatus status = asynSuccess;
  unsigned axisNo = pAxis->axisNo_;

  pAxis->setStringParam(EthercatMCreason27_, "High limit");
  pAxis->setStringParam(EthercatMCreason26_, "Low limit");
  pAxis->setStringParam(EthercatMCreason25_,  "Dynamic problem, timeout");
  pAxis->setStringParam(EthercatMCreason24_,  "Static problem, inhibit");
#if 0
  pAxis->setStringParam(EthercatMCaux7_,  "Aux 7");
  pAxis->setStringParam(EthercatMCaux6_,  "Aux 6");
  pAxis->setStringParam(EthercatMCaux5_,  "Aux 5");
  pAxis->setStringParam(EthercatMCaux4_,  "Aux 4");
  pAxis->setStringParam(EthercatMCaux3_,  "Aux 3");
  pAxis->setStringParam(EthercatMCaux2_,  "Aux 2");
  pAxis->setStringParam(EthercatMCaux1_,  "Aux 1");
  pAxis->setStringParam(EthercatMCaux0_,  "Aux 0");
#endif
  /* AUX bits */
  {
    unsigned auxBitIdx = 0;
    for (auxBitIdx = 0; auxBitIdx <= 23; auxBitIdx++) {
      if ((iAllFlags >> auxBitIdx) & 1) {
        char auxBitName[34];
        unsigned infoType16 = 16;
        int functionNo = EthercatMCaux0_ + auxBitIdx;
        memset(&auxBitName, 0, sizeof(auxBitName));
        status = readDeviceIndexer(devNum, infoType16 + auxBitIdx);
        if (status) return status;
        status = getPlcMemoryString(ctrlLocal.indexerOffset + 1*2,
                                    auxBitName,
                                    sizeof(auxBitName));
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sauxBitName[%d] auxBitName(%02u)=%s\n",
                  modNamEMC, axisNo, auxBitIdx, auxBitName);
        if (status) return status;
        if ((functionNo >= 0) && (functionNo <= EthercatMCaux7_)) {
          pAxis->setStringParam(functionNo, auxBitName);
        }
        if (!strcmp("notHomed", auxBitName)) {
          pAxis->setAuxBitsNotHomedMask(1 << auxBitIdx);
        }
      }
    }
  }
  /* Limits */
  setDoubleParam(axisNo, EthercatMCCfgPMAX_RB_, fAbsMax);
  setDoubleParam(axisNo, EthercatMCCfgPMIN_RB_, fAbsMin);

#ifdef motorHighLimitROString
  udateMotorLimitsRO(axisNo,
                     (fAbsMin > fABSMIN && fAbsMax < fABSMAX),
                     fAbsMax,
                     fAbsMin);
#endif

  return status;
}

asynStatus EthercatMCController::initialPollIndexer(void)
{
  asynStatus status;
  uint32_t iTmpVer = 0xC0DEAFFE;
  double version = 0.0;
  struct {
    char desc[34];
    char vers[34];
    char author1[34];
    char author2[34];
  } descVersAuthors;
  unsigned devNum;
  unsigned infoType0 = 0;
  unsigned infoType4 = 4;
  unsigned infoType5 = 5;
  unsigned infoType6 = 6;
  unsigned infoType7 = 7;
  int      axisNo = 0;

  memset(&descVersAuthors, 0, sizeof(descVersAuthors));
  if (!ctrlLocal.adsport) {
    ctrlLocal.adsport = 851;
  }
  status = getPlcMemoryUint(0, &iTmpVer, sizeof(iTmpVer));
  if (status) return status;

  if (iTmpVer == 0x44fbe0a4) {
    version = 2015.02;
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sadsport=%u version=%f\n",
            modNamEMC, ctrlLocal.adsport, version);

  if (!version) status = asynDisabled;
  if (status) goto endPollIndexer;

  ctrlLocal.indexerOffset = 4;
  status = getPlcMemoryUint(ctrlLocal.indexerOffset,
                            &ctrlLocal.indexerOffset, 2);
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sindexerOffset=%u\n",
            modNamEMC, ctrlLocal.indexerOffset);

  for (devNum = 0; devNum < 100; devNum++) {
    unsigned iTypCode = -1;
    unsigned iSize = -1;
    unsigned iOffset = -1;
    unsigned iUnit = -1;
    unsigned iAllFlags = -1;
    double fAbsMin = 0;
    double fAbsMax = 0;
    status = readDeviceIndexer(devNum, infoType0);
    if (!status) {
      struct {
        uint8_t   typCode_0;
        uint8_t   typCode_1;
        uint8_t   size_0;
        uint8_t   size_1;
        uint8_t   offset_0;
        uint8_t   offset_1;
        uint8_t   unit_0;
        uint8_t   unit_1;
        uint8_t   flags_0;
        uint8_t   flags_1;
        uint8_t   flags_2;
        uint8_t   flags_3;
        uint8_t   absMin[4];
        uint8_t   absMax[4];
      } infoType0_data;
      status = getPlcMemoryViaADS(ctrlLocal.indexerOffset +  1*2,
                                  &infoType0_data, sizeof(infoType0_data));
      if (!status) {
        iTypCode  = infoType0_data.typCode_0 + (infoType0_data.typCode_1 << 8);
        iSize     = infoType0_data.size_0 + (infoType0_data.size_1 << 8);
        iOffset   = infoType0_data.offset_0 + (infoType0_data.offset_1 << 8);
        iUnit     = infoType0_data.unit_0 + (infoType0_data.unit_1 << 8);
        iAllFlags = infoType0_data.flags_0 + (infoType0_data.flags_1 << 8) +
          (infoType0_data.flags_2 << 16) + (infoType0_data.flags_3 << 24);
        fAbsMin   = netToDouble(&infoType0_data.absMin,
                                sizeof(infoType0_data.absMin));
        fAbsMax   = netToDouble(&infoType0_data.absMax,
                                sizeof(infoType0_data.absMax));
      }
    }
    status = readDeviceIndexer(devNum, infoType4);
    if (!status) {
      getPlcMemoryString(ctrlLocal.indexerOffset + 1*2,
                         descVersAuthors.desc,
                         sizeof(descVersAuthors.desc));
    }
    status = readDeviceIndexer(devNum, infoType5);
    if (!status) {
      getPlcMemoryString(ctrlLocal.indexerOffset + 1*2,
                         descVersAuthors.vers,
                         sizeof(descVersAuthors.vers));
    }
    status = readDeviceIndexer(devNum, infoType6);
    if (!status) {
      getPlcMemoryString(ctrlLocal.indexerOffset + 1*2,
                         descVersAuthors.author1,
                         sizeof(descVersAuthors.author1));
    }
    status = readDeviceIndexer(devNum, infoType7);
    if (!status) {
      getPlcMemoryString(ctrlLocal.indexerOffset + 1*2,
                         descVersAuthors.author2,
                         sizeof(descVersAuthors.author2));
    }
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%sindexerDevice Offset=%u %20s "
              "TypCode=0x%x Size=%u UnitCode=0x%x AllFlags=0x%x AbsMin=%e AbsMax=%e\n",
              modNamEMC, iOffset, descVersAuthors.desc,
              iTypCode, iSize, iUnit, iAllFlags, fAbsMin, fAbsMax);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%sdescVersAuthors(%d)  vers=%s author1=%s author2=%s\n",
              modNamEMC, axisNo,
              descVersAuthors.vers,
              descVersAuthors.author1,
              descVersAuthors.author2);
    if (!iTypCode && !iSize && !iOffset) {
      break; /* End of list ?? */
    }
    switch (iTypCode) {
    case 0x5008:
    case 0x500C:
    case 0x5010:
      {
        char unitCodeTxt[40];
        EthercatMCIndexerAxis *pAxis;
        axisNo++;
        pAxis = static_cast<EthercatMCIndexerAxis*>(asynMotorController::getAxis(axisNo));
        if (!pAxis) {
          pAxis = new EthercatMCIndexerAxis(this, axisNo);
        }
        /* Now we have an axis */

        status = newIndexerAxis(pAxis,
                                devNum,
                                iAllFlags,
                                fAbsMin,
                                fAbsMax,
                                iOffset);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sTypeCode(%d) iTypCode=%x pAxis=%p status=%s (%d)\n",
                  modNamEMC, axisNo, iTypCode, pAxis,
                  EthercatMCstrStatus(status), (int)status);
        if (status) goto endPollIndexer;

        pAxis->setIndexerDevNumOffsetTypeCode(devNum, iOffset, iTypCode);
        setStringParam(axisNo,  EthercatMCCfgDESC_RB_, descVersAuthors.desc);
        snprintf(unitCodeTxt, sizeof(unitCodeTxt), "%s%s",
                 plcUnitPrefixTxt(( (int8_t)((iUnit & 0xFF00)>>8))),
                 plcUnitTxtFromUnitCode(iUnit & 0xFF));
        setStringParam(axisNo,  EthercatMCCfgEGU_RB_, unitCodeTxt);
      }
    }
  }

 endPollIndexer:
  /* Special case: asynDisabled means "no indexer found".
     That is OK, return asynSuccess */
  if (status == asynDisabled)
    return asynSuccess;
  return status;

}
