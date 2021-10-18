/*
  FILENAME... ethercatmcIndexer.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "ethercatmcController.h"
#include "ethercatmcIndexerAxis.h"

#include <epicsThread.h>

/* Alarm definition from EPICS Base */
#include <alarm.h>

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif


/* Sleep time and max counter for communication*/
#define MAX_COUNTER 14

/*
 * Calculation of sleep time when the PLC answers/answers with "interface busy"
 * or retry later
 */
extern "C" {
  double calcSleep(int counter)
  {
    static double sleepTime = 0.005; /* 5 msec */
    return sleepTime * (counter <<1);
  }
};

extern "C" {
  const char *plcUnitTxtFromUnitCode(unsigned unitCode)
  {
    const static char *const unitTxts[] = {
      "",
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


extern "C" {
  const char *paramIfCmdToString(unsigned cmdSubParamIndex)
  {
    switch (cmdSubParamIndex & PARAM_IF_CMD_MASK) {
    case PARAM_IF_CMD_INVALID         : return "PARAM_IF_CMD_INVALID";
    case PARAM_IF_CMD_DOREAD          : return "PARAM_IF_CMD_DOREAD";
    case PARAM_IF_CMD_DOWRITE         : return "PARAM_IF_CMD_DOWRITE";
    case PARAM_IF_CMD_BUSY            : return "PARAM_IF_CMD_BUSY";
    case PARAM_IF_CMD_DONE            : return "PARAM_IF_CMD_DONE";
    case PARAM_IF_CMD_ERR_NO_IDX      : return "PARAM_IF_CMD_ERR_NO_IDX";
    case PARAM_IF_CMD_ERR_READONLY    : return "PARAM_IF_CMD_ERR_READONLY";
    case PARAM_IF_CMD_ERR_RETRY_LATER : return "PARAM_IF_CMD_ERR_RETRY_LATER";
    default: return "cmdSubParamIndexXXXX";
    }
  }
}

extern "C" {
  const char *plcParamIndexTxtFromParamIndex(unsigned cmdSubParamIndex)
 {
   switch(cmdSubParamIndex & PARAM_IF_IDX_MASK) {
   case 0:                                 return "PARAM_ZERO";
   case PARAM_IDX_OPMODE_AUTO_UINT:        return "OPMODE_AUTO";
   case PARAM_IDX_MICROSTEPS_UINT:         return "MICROSTEPS";
   case PARAM_IDX_ABS_MIN_FLOAT:           return "ABS_MIN";
   case PARAM_IDX_ABS_MAX_FLOAT:           return "ABS_MAX";
   case PARAM_IDX_USR_MIN_FLOAT:           return "USR_MIN";
   case PARAM_IDX_USR_MAX_FLOAT:           return "USR_MAX";
   case PARAM_IDX_WRN_MIN_FLOAT:           return "WRN_MIN";
   case PARAM_IDX_WRN_MAX_FLOAT:           return "WRN_MAX";
   case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT: return "FOLLOWING_ERR_WIN";
   case PARAM_IDX_HYTERESIS_FLOAT:         return "HYTERESIS";
   case PARAM_IDX_REFSPEED_FLOAT:          return "REFSPEED";
   case PARAM_IDX_VBAS_FLOAT:              return "VBAS";
   case PARAM_IDX_SPEED_FLOAT:             return "SPEED";
   case PARAM_IDX_ACCEL_FLOAT:             return "ACCEL";
   case PARAM_IDX_IDLE_CURRENT_FLOAT:      return "IDLE_CURRENT";
   case PARAM_IDX_MOVE_CURRENT_FLOAT:      return "MOVE_CURRENT";
   case PARAM_IDX_MICROSTEPS_FLOAT:        return "MICROSTEPS";
   case PARAM_IDX_STEPS_PER_UNIT_FLOAT:    return "STEPS_PER_UNIT";
   case PARAM_IDX_HOME_POSITION_FLOAT:     return "HOME_POSITION";
   case PARAM_IDX_FUN_REFERENCE:           return "REFERENCE";
   case PARAM_IDX_FUN_SET_POSITION:        return "SET_POSITION";
   case PARAM_IDX_FUN_MOVE_VELOCITY:       return "MOVE_VELOCITY";
   case PARAM_IDX_USR_MIN_EN_UINT:         return "USR_MIN_EN_OLD";
   case PARAM_IDX_USR_MAX_EN_UINT:         return "USR_MAX_EN_OLD";
   case PARAM_IDX_HOMPROC_UINT:            return "HOMPROC_OLD";
   case PARAM_IDX_USR_MIN_EN_FLOAT:        return "USR_MIN_EN";
   case PARAM_IDX_USR_MAX_EN_FLOAT:        return "USR_MAX_EN";
   case PARAM_IDX_HOMPROC_FLOAT:           return "HOMPROC";
   case PARAM_IDX_UNITS_PER_REV_FLOAT:     return "UNITS_PER_REV";
   case PARAM_IDX_STEPS_PER_REV_FLOAT:     return "STEPS_PER_REV";
   case PARAM_IDX_MAX_VELO_FLOAT:          return "MAX_VELO";
   default: return "PX";
   }
 }
};

extern "C" {
  int paramIndexIsInteger(unsigned paramIndex)
  {
    if (paramIndex < 30) {
      /* parameters below 30 are unsigned integers in the PLC */
      return 1;
    } else if (paramIndex >= 192 && paramIndex <= 200) {
      /* Parameters 192 .. 200 are integers as well */
      return 1;
    } else {
      return 0;
    }
  }
  int paramIndexIsMovingFunction(unsigned paramIndex) {
    switch (paramIndex) {
    case PARAM_IDX_FUN_MOVE_VELOCITY:
    case PARAM_IDX_FUN_REFERENCE:
      return 1;
    default:
      return 0;
    }
  }
  int paramIndexIsParameterToPoll(unsigned paramIndex) {
    if (paramIndex == PARAM_IDX_OPMODE_AUTO_UINT) {
      return 0; /* parameter 0 is power on only for the moment. Don't poll it */
    } else if (paramIndex < PARAM_IF_IDX_FIRST_FUNCTION) {
      return 1;
    } else if ((paramIndex >= PARAM_IF_IDX_FIRST_CUSTOM_PARA &&
                paramIndex <= PARAM_IF_IDX_LAST_CUSTOM_PARA)) {
      return 1;
    }
    /* All others are not pollable */
    return 0;
  }
  /*
   * These parameters are slow to read -
   * poll them later in the background polling
   * Note: This is probably a temporary workaround
   * In the log run, reading of all paramters should be fast
   */
  int paramIndexIsReadLaterInBackground(unsigned paramIndex) {
    switch (paramIndex) {
    case PARAM_IDX_UNITS_PER_REV_FLOAT: return 1;
    case PARAM_IDX_STEPS_PER_REV_FLOAT: return 1;
    default:
      return 0;
    }
  }
};

static const double fABSMIN = -3.0e+38;
static const double fABSMAX =  3.0e+38;

/* Wrapper methods: if something fails, change state */
asynStatus
ethercatmcController::getPlcMemoryOnErrorStateChangeFL(unsigned indexOffset,
                                                       void *data,
                                                       size_t lenInPlc,
                                                       const char *fileName,
                                                       int lineNo)
{
  asynStatus status;
  status = getPlcMemoryViaADSFL(indexOffset, data, lenInPlc,
                                fileName, lineNo);
  if (status) {
    asynPrint(pasynUserController_,
              ASYN_TRACE_FLOW,
              "%sgetPlcMemoryOnErrorStateChangeFL %s:%d status=%s (%d) oldStatus=%d\n",
              modNamEMC, fileName, lineNo,
              ethercatmcstrStatus(status), (int)status,
              (int)ctrlLocal.oldStatus);
    handleStatusChange(status);
  }
  return status;
}

asynStatus
ethercatmcController::setPlcMemoryOnErrorStateChangeFL(unsigned indexOffset,
                                                       const void *data,
                                                       size_t lenInPlc,
                                                       const char *fileName,
                                                       int lineNo)
{
  asynStatus status;
  status = setPlcMemoryViaADSFL(indexOffset, data, lenInPlc,
                                fileName, lineNo);
  if (status) {
    asynPrint(pasynUserController_,
              ASYN_TRACE_FLOW,
              "%ssetPlcMemoryOnErrorStateChangeFL %s:%d status=%s (%d) oldStatus=%d\n",
              modNamEMC, fileName, lineNo,
              ethercatmcstrStatus(status), (int)status,
              (int)ctrlLocal.oldStatus);
    handleStatusChange(status);
  }
  return status;

}
/* end of wrappers */

/* Re-define calles without FILE and LINE */
#define getPlcMemoryOnErrorStateChange(a,b,c)  getPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)
#define setPlcMemoryOnErrorStateChange(a,b,c)  setPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)

/* No "direct" calls into ADS below this point */
#undef getPlcMemoryViaADS
#undef setPlcMemoryViaADS
#define getPlcMemoryViaADS    #error
#define setPlcMemoryViaADS    #error
#define getPlcMemoryViaADSFL  #error
#define setPlcMemoryViaADSFL  #error


asynStatus ethercatmcController::getPlcMemoryUintFL(unsigned indexOffset,
                                                    unsigned *value,
                                                    size_t lenInPlc,
                                                    const char *fileName,
                                                    int lineNo)
{
  asynStatus status;

  memset(value, 0, lenInPlc);
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    status = getPlcMemoryOnErrorStateChangeFL(indexOffset, &raw, lenInPlc, fileName, lineNo);
    *value = netToUint(&raw, lenInPlc);
    return status;
  }
  return asynError;
}

asynStatus ethercatmcController::setPlcMemoryInteger(unsigned indexOffset,
                                                     int value,
                                                     size_t lenInPlc)
{
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    uintToNet(value, &raw, lenInPlc);
    return setPlcMemoryOnErrorStateChange(indexOffset, raw, lenInPlc);
  } else {
    return asynError;
  }
}


asynStatus ethercatmcController::getPlcMemoryDouble(unsigned indexOffset,
                                                    double *value,
                                                    size_t lenInPlc)
{
  asynStatus status;

  memset(value, 0, lenInPlc);
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    status = getPlcMemoryOnErrorStateChange(indexOffset, &raw, lenInPlc);
    *value = netToDouble(&raw, lenInPlc);
    return status;
  }
  return asynError;
}

asynStatus ethercatmcController::setPlcMemoryDouble(unsigned indexOffset,
                                                    double value,
                                                    size_t lenInPlc)
{
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    doubleToNet(value, &raw, lenInPlc);
    return setPlcMemoryOnErrorStateChange(indexOffset, &raw, lenInPlc);
  } else {
    return asynError;
  }
}

asynStatus ethercatmcController::setSAFIntegerOnAxisViaADSFL(unsigned indexGroup,
                                                             unsigned indexOffset,
                                                             int      value,
                                                             size_t   lenInPlc,
                                                             const char *fileName,
                                                             int lineNo)
{
  const static unsigned targetAdsport = 501;
  asynStatus status;
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    uintToNet(value, &raw, lenInPlc);

    status = setMemIdxGrpIdxOffFL(indexGroup,
                                  indexOffset,
                                  targetAdsport,
                                  &raw, lenInPlc,
                                  fileName, lineNo);
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%s%s:%d setSAFIntegerOnAxisViaADSFL indexGroup=0x%X indexOffset=0x%X"
              " value=%d lenInPlc=%u status=%s (%d)\n",
              modNamEMC,fileName, lineNo,
              indexGroup, indexOffset,
              value,  (unsigned)lenInPlc,
              ethercatmcstrStatus(status), (int)status);
    return status;
  } else {
    return asynError;
  }
}

asynStatus ethercatmcController::setSAFDoubleOnAxisViaADSFL(unsigned indexGroup,
                                                            unsigned indexOffset,
                                                            double   value,
                                                            size_t   lenInPlc,
                                                            const char *fileName,
                                                            int lineNo)
{
  const static unsigned targetAdsport = 501;
  asynStatus status;
  if (lenInPlc == 8 || lenInPlc == 4) {
    uint8_t raw[8];
    doubleToNet(value, &raw, lenInPlc);

    status = setMemIdxGrpIdxOffFL(indexGroup,
                                  indexOffset,
                                  targetAdsport,
                                  &raw, lenInPlc,
                                  fileName, lineNo);
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%s%s:%d setSAFIntegerOnAxisViaADSFL indexGroup=0x%X indexOffset=0x%X"
              " value=%f lenInPlc=%u status=%s (%d)\n",
              modNamEMC,fileName, lineNo,
              indexGroup, indexOffset,
              value,  (unsigned)lenInPlc,
              ethercatmcstrStatus(status), (int)status);
    return status;
  } else {
    return asynError;
  }
}


asynStatus ethercatmcController::readDeviceIndexerFL(unsigned devNum,
                                                     unsigned infoType,
                                                     const char *fileName,
                                                     int lineNo)
{
  asynStatus status;
  unsigned value = (devNum + (infoType << 8));
  unsigned valueAcked = 0x8000 + value;
  unsigned counter = 0;
  if ((devNum > 0xFF) || (infoType > 0xFF)) {
    status = asynDisabled;
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%s%s:%d readDeviceIndexer devNum=%u infoType=%u status=%s (%d)\n",
              modNamEMC, fileName, lineNo, devNum, infoType,
              ethercatmcstrStatus(status), (int)status);
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
              "%s%s:%d readDeviceIndexer status=%s (%d)\n",
              modNamEMC,fileName, lineNo,
              ethercatmcstrStatus(status), (int)status);
    return status;
  }
  while (counter < MAX_COUNTER) {
    status = getPlcMemoryUint(ctrlLocal.indexerOffset, &value, 2);
    if (status) {
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%s%s:%d readDeviceIndexer status=%s (%d)\n",
                modNamEMC, fileName, lineNo,
                ethercatmcstrStatus(status), (int)status);
      return status;
    }
    if (value == valueAcked) return asynSuccess;
    counter++;
    epicsThreadSleep(calcSleep(counter));
  }
  status = asynDisabled;
  asynPrint(pasynUserController_,
            ASYN_TRACE_INFO,
            "%sreadDeviceIndexer devNum=0x%X infoType=0x%X counter=%u value=0x%X status=%s (%d)\n",
            modNamEMC, devNum, infoType, counter, value,
            ethercatmcstrStatus(status), (int)status);
  return status;

}

asynStatus ethercatmcController::indexerWaitSpecialDeviceIdle(unsigned indexOffset)
{
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned   ctrlLen = 0;
  unsigned   counter = 0;
  unsigned   plcNotHostHasWritten;

  while (counter < MAX_COUNTER) {
    status = getPlcMemoryUint(indexOffset, &ctrlLen, 2);
    plcNotHostHasWritten = (ctrlLen & 0x8000) ? 1 : 0;

    asynPrint(pasynUserController_,
              status ? traceMask | ASYN_TRACE_INFO : traceMask,
              "%sindexerWaitSpecialDeviceIdle ctrlLen=0x%04x status=%s (%d)\n",
              modNamEMC, ctrlLen,
              ethercatmcstrStatus(status), (int)status);
    if (status) return status;
    if (plcNotHostHasWritten) return asynSuccess;
    counter++;
    epicsThreadSleep(calcSleep(counter));
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sindexOffset=%u ctrlLen=0x%04X counter=%d\n",
            modNamEMC, indexOffset, ctrlLen, counter);
  return asynDisabled;
}

asynStatus ethercatmcController::indexerParamReadFL(int axisNo,
                                                    unsigned paramIfOffset,
                                                    unsigned paramIndex,
                                                    unsigned lenInPlcPara,
                                                    double   *value,
                                                    const char *fileName,
                                                    int lineNo)
{
  paramIf_type paramIf_from_MCU;
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned cmd      = PARAM_IF_CMD_DOREAD + paramIndex;
  size_t lenInPLCparamIf = sizeof(paramIf_from_MCU.paramCtrl) + lenInPlcPara;
  unsigned counter = 0;

  if (paramIndex > 0xFF ||
      lenInPlcPara > sizeof(paramIf_from_MCU.paramValueRaw)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s paramIndex=%u lenInPlcPara=%u\n",
              modNamEMC, paramIndex, lenInPlcPara);
    return asynDisabled;
  }
  while (counter < MAX_COUNTER) {
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    status = getPlcMemoryOnErrorStateChange(paramIfOffset,
                                            &paramIf_from_MCU,
                                            lenInPLCparamIf);

    if (status) return status;
    unsigned cmdSubParamIndexRB = NETTOUINT(paramIf_from_MCU.paramCtrl);
    unsigned paramIndexRB = cmdSubParamIndexRB & PARAM_IF_IDX_MASK;
    if (counter > 1) {
      asynPrint(pasynUserController_, traceMask | ASYN_TRACE_INFO,
                "%s:%d %s(%d) paramIfOffset=%u paramIdxFunction=%s (%u 0x%02X) "
                "counter=%u cmdSubParamIndexRB=%s (0x%04X)\n",
                fileName, lineNo, "indexerParamRead", axisNo, paramIfOffset,
                plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                counter,
                paramIfCmdToString(cmdSubParamIndexRB), cmdSubParamIndexRB);
    }
    switch (cmdSubParamIndexRB & PARAM_IF_CMD_MASK) {
    case PARAM_IF_CMD_DONE:
      if (paramIndexRB == paramIndex) {
        /* This is good, return */
        double fValue;
        if (paramIndexIsInteger(paramIndex)) {
          fValue = (double)netToUint(&paramIf_from_MCU.paramValueRaw,
                                     lenInPlcPara);
        } else {
          fValue = netToDouble(&paramIf_from_MCU.paramValueRaw,
                               lenInPlcPara);
        }
        *value = fValue;
        return asynSuccess;
      }
      /* fall through */
    case PARAM_IF_CMD_ERR_NO_IDX:
    case PARAM_IF_CMD_ERR_READONLY:
    case PARAM_IF_CMD_ERR_RETRY_LATER:
      /* param interface is not busy */
      if (paramIndexRB != paramIndex) {
        /* Send the read request */
        status = setPlcMemoryInteger(paramIfOffset, cmd,
                                     (unsigned)sizeof(paramIf_from_MCU.paramCtrl));
        if (status) return status;
      } else {
        return asynDisabled;
      }
      break;
    case PARAM_IF_CMD_INVALID:
    case PARAM_IF_CMD_BUSY:
    case PARAM_IF_CMD_DOREAD:
    case PARAM_IF_CMD_DOWRITE:
      /* wait */
      break;
    }
    epicsThreadSleep(calcSleep(counter));
    counter++;
  }
  return asynError;
}

asynStatus ethercatmcController::indexerParamWrite(int axisNo,
                                                   unsigned paramIfOffset,
                                                   unsigned paramIndex,
                                                   unsigned lenInPlcPara,
                                                   double value,
                                                   double *pValueRB)
{
  ethercatmcIndexerAxis *pAxis = static_cast<ethercatmcIndexerAxis*>(asynMotorController::getAxis(axisNo));
  paramIf_type paramIf_to_MCU;
  paramIf_type paramIf_from_MCU;
  unsigned traceMask = ASYN_TRACE_INFO;
  asynStatus status = asynSuccess;
  unsigned cmd      = PARAM_IF_CMD_DOWRITE + paramIndex;
  size_t lenInPLCparamIf = sizeof(paramIf_to_MCU.paramCtrl) + lenInPlcPara;
  unsigned counter = 0;
  int has_written = 0;

  if (pAxis) {
    if (pAxis->drvlocal.PILSparamPerm[paramIndex] == PILSparamPermRead) {
      return asynParamWrongType;
    } else if (pAxis->drvlocal.PILSparamPerm[paramIndex] == PILSparamPermNone) {
      return asynParamBadIndex;
    }
  }
  if (paramIndex > 0xFF ||
      lenInPlcPara > sizeof(paramIf_to_MCU.paramValueRaw)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s paramIndex=%u lenInPlcPara=%u\n",
              modNamEMC, paramIndex, lenInPlcPara);
    return asynDisabled;
  }
  memset(&paramIf_to_MCU, 0, sizeof(paramIf_to_MCU));
  memset(&paramIf_from_MCU, 0, sizeof(paramIf_from_MCU));
  if (paramIndexIsInteger(paramIndex))
    uintToNet((int)value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  else
    doubleToNet(value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  UINTTONET(cmd, paramIf_to_MCU.paramCtrl);

  while (counter < MAX_COUNTER) {
    double valueRB = -1.0;
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    status = getPlcMemoryOnErrorStateChange(paramIfOffset,
                                            &paramIf_from_MCU,
                                            lenInPLCparamIf);
    if (paramIndexIsInteger(paramIndex)) {
      valueRB = netToUint(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    } else {
      valueRB = netToDouble(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    }

    if (status) return status;
    unsigned cmdSubParamIndexRB = NETTOUINT(paramIf_from_MCU.paramCtrl);
    unsigned paramIndexRB = cmdSubParamIndexRB & PARAM_IF_IDX_MASK;

    if (counter >= 1) {
      asynPrint(pasynUserController_, traceMask,
                "%sindexerParamWrite(%d) paramIndex=%s(%u 0x%02X) value=%02g "
                "counter=%u cmdSubParamIndexRB=%s (0x%04X)\n",
                modNamEMC, axisNo,
                plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                value, counter,
                paramIfCmdToString(cmdSubParamIndexRB), cmdSubParamIndexRB);
    }
    unsigned paramIfCmd = cmdSubParamIndexRB & PARAM_IF_CMD_MASK;
    switch (paramIfCmd) {
    case PARAM_IF_CMD_DONE:
      {
        if (paramIndexRB == paramIndex) {
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) paramIndex=%s(%u) value=%02g valueRB=%02g has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                    value, valueRB, has_written);
          if (paramIndexIsMovingFunction(paramIndex)) {
            /* New param interface handling:
               PLC goes to DONE, the interface is released
               Since this is a moving function, value != valueRB is OK here */
            if (has_written) {
              if (pValueRB) *pValueRB = valueRB;
              return asynSuccess;
            }
          } else {
            if (value == valueRB || has_written) {
              if (pValueRB) *pValueRB = valueRB;
              return asynSuccess;
            }
          }
        } else {
          has_written = 0;
        }
      }
      /* fall through */
    case PARAM_IF_CMD_ERR_NO_IDX:
    case PARAM_IF_CMD_ERR_READONLY:
    case PARAM_IF_CMD_ERR_RETRY_LATER:
      {
        /* param interface is not busy */
        if (paramIndexRB != paramIndex || !has_written) {
          /* Send the write request, unless we already done it */
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) writing to MCU paramIndex=%s(%u) value=%02g lenInPlcPara=%u has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                    value, lenInPlcPara, has_written);
          status = setPlcMemoryOnErrorStateChange(paramIfOffset, &paramIf_to_MCU,
                                                  (unsigned)sizeof(paramIf_to_MCU));
          if (status) return status;
          has_written = 1;
        } else if (paramIndexRB == paramIndex) {
          if (pAxis) {
            if (paramIfCmd == PARAM_IF_CMD_ERR_NO_IDX) {
              pAxis->drvlocal.PILSparamPerm[paramIndex] = PILSparamPermNone;
              return asynParamBadIndex;
            } else if (paramIfCmd == PARAM_IF_CMD_ERR_READONLY) {
              pAxis->drvlocal.PILSparamPerm[paramIndex] = PILSparamPermRead;
              return asynParamWrongType;
            }
          }
          return asynDisabled;
        }
      }
      break;
    case PARAM_IF_CMD_BUSY:
      {
        /* A "function" goes into busy - and stays there */
        /* No parameter settings during jogging/homing */
        if (paramIndexIsMovingFunction(paramIndexRB)) {
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) paramIndex=%s(%u 0x%02X) value=%02g "
                    "cmdSubParamIndexRB=%s (0x%04X)\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                    value,
                    paramIfCmdToString(cmdSubParamIndexRB), cmdSubParamIndexRB);
          if (paramIndexRB == paramIndex) {
            /* "our" function: return */
            if (pValueRB) *pValueRB = valueRB;
            return asynSuccess;
          }
          return asynDisabled;
        }
      }
      /* fall through */
    case PARAM_IF_CMD_INVALID:
    case PARAM_IF_CMD_DOREAD:
    case PARAM_IF_CMD_DOWRITE:
      /* wait */
      break;
    }
    epicsThreadSleep(calcSleep(counter));
    if (has_written) {
      counter++;
    }
  }
  asynPrint(pasynUserController_,
            traceMask | ASYN_TRACE_INFO,
            "%scounter=%u\n",
            modNamEMC, counter);
  return asynDisabled;
}

asynStatus
ethercatmcController::getPlcMemoryFromProcessImage(unsigned indexOffset,
                                                   void *data,
                                                   size_t lenInPlc)
{
  if (ctrlLocal.pIndexerProcessImage &&
      ctrlLocal.lastDeviceEndOffset) {
    memcpy(data,
           &ctrlLocal.pIndexerProcessImage[indexOffset],
           lenInPlc);
    return asynSuccess;
  }
  /* no process image, get values */
  return asynDisabled;
}

void ethercatmcController::parameterFloatReadBack(unsigned axisNo,
                                                  int initial,
                                                  unsigned paramIndex,
                                                  double fValue)
{
  asynMotorAxis *pAxis=getAxis((int)axisNo);
  const static double fullsrev = 200;    /* (default) Full steps/revolution */

  switch(paramIndex) {
  case PARAM_IDX_OPMODE_AUTO_UINT:
    /* CNEN for EPICS */
    pAxis->setIntegerParam(motorStatusGainSupport_, 1);
    break;
  case PARAM_IDX_MICROSTEPS_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgSREV_RB_, fullsrev * fValue, "srev");
    break;
  case PARAM_IDX_USR_MIN_FLOAT:
    {
      updateCfgValue(axisNo, ethercatmcCfgDLLM_RB_,   fValue, "dllm");
      if (initial) updateCfgValue(axisNo, ethercatmcCfgDLLM_,   fValue, "CfgDllm");
    }
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_ABS_MIN_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgPMIN_RB_, fValue, "posmin");
    break;
  case PARAM_IDX_ABS_MAX_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgPMAX_RB_, fValue, "posmax");
    break;
  case PARAM_IDX_USR_MAX_FLOAT:
    {
      updateCfgValue(axisNo, ethercatmcCfgDHLM_RB_, fValue, "dhlm");
      if (initial) updateCfgValue(axisNo, ethercatmcCfgDHLM_,   fValue, "CfgDhlm");
    }
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_WRN_MIN_FLOAT:
    break;
  case PARAM_IDX_WRN_MAX_FLOAT:
    break;
  case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgPOSLAG_RB_, fValue, "poslag");
    updateCfgValue(axisNo, ethercatmcCfgPOSLAG_En_RB_, 1, "poslag_en");
    break;
  case PARAM_IDX_HYTERESIS_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgSPDB_RB_, fValue, "spdb");
    updateCfgValue(axisNo, ethercatmcCfgRDBD_RB_, fValue, "rdbd");
    updateCfgValue(axisNo, ethercatmcCfgRDBD_En_RB_, 1, "rdbd_en");
    break;
  case PARAM_IDX_REFSPEED_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgHVEL_RB_, fValue, "hvel");
    break;
  case PARAM_IDX_SPEED_FLOAT:
    if (initial) updateCfgValue(axisNo, ethercatmcCfgVELO_RB_, fValue, "veloCFG");
    updateCfgValue(axisNo, ethercatmcVel_RB_, fValue, "veloRB");
#ifdef motorDefVelocityROString
    pAxis->setDoubleParam(motorDefVelocityRO_, fValue);
#endif
    break;
  case PARAM_IDX_ACCEL_FLOAT:
    if (initial) updateCfgValue(axisNo, ethercatmcCfgACCS_RB_, fValue, "accsRB");
    updateCfgValue(axisNo, ethercatmcAcc_RB_, fValue, "accsRB");
    break;
  case PARAM_IDX_IDLE_CURRENT_FLOAT:
    break;
  case PARAM_IDX_MOVE_CURRENT_FLOAT:
    break;
  case PARAM_IDX_MICROSTEPS_UINT:
    break;
  case PARAM_IDX_STEPS_PER_UNIT_FLOAT:
    {
      double urev = fabs(fullsrev / fValue);
      updateCfgValue(axisNo, ethercatmcCfgUREV_RB_, urev, "urev");
    }
    break;
  case PARAM_IDX_HOME_POSITION_FLOAT:
    updateCfgValue(axisNo, ethercatmcHomPos_RB_, fValue, "homPosRB");
    if (initial) updateCfgValue(axisNo, ethercatmcHomPos_,   fValue, "hompos");
    break;
  case PARAM_IDX_FUN_REFERENCE:
#ifdef  motorNotHomedProblemString
    pAxis->setIntegerParam(motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
    updateCfgValue(axisNo, ethercatmcHomeVis_, 1, "homeVis");
    break;
  case PARAM_IDX_FUN_SET_POSITION:
#ifdef  motorNotHomedProblemString
    pAxis->setIntegerParam(motorNotHomedProblem_, MOTORNOTHOMEDPROBLEM_ERROR);
#endif
    updateCfgValue(axisNo, ethercatmcFoffVis_, 1, "foffVis");
    break;
  case PARAM_IDX_FUN_MOVE_VELOCITY:
    if (initial) updateCfgValue(axisNo, ethercatmcCfgJVEL_RB_, fabs(fValue), "jvel");
    break;
  case PARAM_IDX_USR_MIN_EN_UINT:
    updateCfgValue(axisNo, ethercatmcCfgDLLM_En_RB_, (int)fValue, "dllm_en");
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_USR_MAX_EN_UINT:
    updateCfgValue(axisNo, ethercatmcCfgDHLM_En_RB_, (int)fValue, "dhlm_en");
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_HOMPROC_UINT:
    updateCfgValue(axisNo, ethercatmcHomProc_RB_, (int)fValue, "homprocRB");
    if (initial) updateCfgValue(axisNo, ethercatmcHomProc_, (int)fValue, "homproc");
    break;
  case PARAM_IDX_USR_MIN_EN_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgDLLM_En_RB_, (int)fValue, "dllm_en");
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_USR_MAX_EN_FLOAT:
    updateCfgValue(axisNo, ethercatmcCfgDHLM_En_RB_, (int)fValue, "dhlm_en");
    udateMotorLimitsRO(axisNo);
    break;
  case PARAM_IDX_HOMPROC_FLOAT:
    updateCfgValue(axisNo, ethercatmcHomProc_RB_, (int)fValue, "homprocRB");
    if (initial) updateCfgValue(axisNo, ethercatmcHomProc_, (int)fValue, "homproc");
    break;
  case PARAM_IDX_UNITS_PER_REV_FLOAT:
    {
      if (fValue > 0.0) {
        double urev = fabs(fValue);
        updateCfgValue(axisNo, ethercatmcCfgUREV_RB_, urev, "urev");
      }
    }
    break;
  case PARAM_IDX_STEPS_PER_REV_FLOAT:
    if (fValue > 0.0) {
      updateCfgValue(axisNo, ethercatmcCfgSREV_RB_, fValue, "srev");
    }
    break;
  case PARAM_IDX_MAX_VELO_FLOAT:
    if (fValue > 0.0) {
      updateCfgValue(axisNo, ethercatmcCfgVMAX_RB_, fValue, "vmax");
    }
    break;
  }
}

asynStatus
ethercatmcController::indexerReadAxisParameters(ethercatmcIndexerAxis *pAxis,
                                                unsigned devNum,
                                                unsigned iOffsBytes,
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
              "%sindexerReadAxisParameters(%d) initialPollDon=%d status=%s (%d)\n",
              modNamEMC, devNum, ctrlLocal.initialPollDone,
              ethercatmcstrStatus(status), (int)status);
    return status;
  }
  for (dataIdx = 0; dataIdx < 16; dataIdx++) {
    unsigned parameters;
    int traceMask = 0;
    parameters = -1;

    status = getPlcMemoryUint(ctrlLocal.indexerOffset + (1 + dataIdx) * 2,
                              &parameters, 2);
    if (status) {
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%sindexerReadAxisParameters (%d) status=%s (%d)\n",
                modNamEMC, axisNo,
                ethercatmcstrStatus(status), (int)status);
      return status;
    }
    /* dataIdx == 0 has ACK + infoType/devNum
       dataIdx == 1 has supported parameters 15..0 */
    asynPrint(pasynUserController_, traceMask,
              "%sparameters[%03u..%03u]=0x%04X\n",
              modNamEMC, dataIdx*16 +15,
              dataIdx*16, parameters);
    /* Where is the parameter interface to this device ?
       See indexerDevice5008interface_type;,
       indexerDevice5010interface_type in indexer.c */
    unsigned paramIfOffset;
    switch (lenInPlcPara) {
    case 4:
      paramIfOffset = iOffsBytes + 10;
      break;
    case 8:
      paramIfOffset = iOffsBytes + 22;
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
      double fValue = 0.0;
      int initial = 1;
      unsigned paramIndex = dataIdx*16 + bitIdx;
      unsigned bitIsSet = parameters & (1 << bitIdx) ? 1 : 0;
      if (bitIsSet) {
        if (paramIndex >= sizeof(pAxis->drvlocal.PILSparamPerm)) {
            asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                      "%sparameters(%d) paramIdx (%u 0x%02X) out of range\n",
                      modNamEMC, axisNo,
                      paramIndex, paramIndex);
            return asynError;
        }
        if (paramIndexIsParameterToPoll(paramIndex) ||
            (paramIndex == PARAM_IDX_FUN_MOVE_VELOCITY)) {
          /* Some parameters are functions: Don't read them.
             tell driver that the function exist
             But read 142, which becomes JVEL */
          if (!paramIndexIsReadLaterInBackground(paramIndex)) {
              status = indexerParamRead(axisNo,
                                        paramIfOffset,
                                        paramIndex,
                                        lenInPlcPara,
                                        &fValue);
              if (status) {
                asynPrint(pasynUserController_,
                          ASYN_TRACE_INFO,
                          "%sindexerReadAxisParameters(%d) paramIdx=%s (%u)"
                          " lenInPlcPara=%u status=%s (%d)\n",
                          modNamEMC, axisNo,
                          plcParamIndexTxtFromParamIndex(paramIndex),
                          paramIndex, lenInPlcPara,
                          ethercatmcstrStatus(status), (int)status);
                return status;
              }
              if (paramIndexIsInteger(paramIndex)) {
                asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                          "%sparameters(%d) paramIdx=%s (%u) iValue=%i\n",
                          modNamEMC, axisNo,
                          plcParamIndexTxtFromParamIndex(paramIndex),
                          paramIndex, (int)fValue);
              } else {
                asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                          "%sparameters(%d) paramIdx=%s (%u) fValue=%f\n",
                          modNamEMC, axisNo,
                          plcParamIndexTxtFromParamIndex(paramIndex),
                          paramIndex, fValue);
              }
            } else {
              asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                        "%sparameters(%d) paramIdx=%s (%u) not polled in background\n",
                        modNamEMC, axisNo,
                        plcParamIndexTxtFromParamIndex(paramIndex),
                        paramIndex);
          }
        }
        pAxis->drvlocal.PILSparamPerm[paramIndex] = PILSparamPermWrite;
        if (paramIndexIsParameterToPoll(paramIndex)) {
          pAxis->addPollNowParam(paramIndex);
        }
        parameterFloatReadBack(axisNo, initial, paramIndex, fValue);
      }
    }
  }
  return asynSuccess;
}

asynStatus
ethercatmcController::newIndexerAxis(ethercatmcIndexerAxis *pAxis,
                                     unsigned devNum,
                                     unsigned iAllFlags,
                                     double   fAbsMin,
                                     double   fAbsMax,
                                     unsigned iOffsBytes)
{
  asynStatus status = asynSuccess;
  unsigned axisNo = pAxis->axisNo_;
  /* AUX bits */
  {
    unsigned auxBitIdx = 0;
    for (auxBitIdx = 0; auxBitIdx < MAX_AUX_BIT_SHOWN; auxBitIdx++) {
      int function = ethercatmcNamAux0_ + auxBitIdx;
      if ((iAllFlags >> auxBitIdx) & 1) {
        char auxBitName[34];
        unsigned infoType16 = 16;
        memset(&auxBitName, 0, sizeof(auxBitName));
        status = readDeviceIndexer(devNum, infoType16 + auxBitIdx);
        if (status) return status;
        status = getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset + 1*2,
                                                auxBitName,
                                                sizeof(auxBitName));
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sauxBitName(%d) auxBitName[%02u]=%s\n",
                  modNamEMC, axisNo, auxBitIdx, auxBitName);
        if (status) return status;
        if (function <= ethercatmcNamAux0_ + MAX_AUX_BIT_SHOWN) {
          pAxis->setStringParam(function, auxBitName);
          setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);
        }
        if (!strcmp("notHomed", auxBitName)) {
          pAxis->setAuxBitsNotHomedMask(1 << auxBitIdx);
        } else if (!strcmp("enabled", auxBitName)) {
          pAxis->setAuxBitsEnabledMask(1 << auxBitIdx);
        } else if (!strcmp("localMode", auxBitName)) {
          pAxis->setAuxBitsLocalModeMask(1 << auxBitIdx);
        } else if (!strcmp("homeSwitch", auxBitName)) {
          pAxis->setAuxBitsHomeSwitchMask(1 << auxBitIdx);
        }
      }
    }
  }
  /* Limits */
  updateCfgValue(axisNo, ethercatmcCfgPMAX_RB_, fAbsMax, "CfgPMAX");
  updateCfgValue(axisNo, ethercatmcCfgPMIN_RB_, fAbsMin, "CfgPMIN");

#ifdef motorHighLimitROString
  udateMotorLimitsRO(axisNo,
                     (fAbsMin > fABSMIN && fAbsMax < fABSMAX),
                     fAbsMax,
                     fAbsMin);
#endif

  return status;
}

asynStatus ethercatmcController::indexerInitialPoll(void)
{
  asynStatus status;
  unsigned firstDeviceStartOffset = (unsigned)-1; /* Will be decreased while we go */
  unsigned lastDeviceEndOffset = 0;  /* will be increased while we go */
  unsigned iTmpVer = 0xC0DEAFFE;
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
  ethercatmcIndexerAxis *pAxis;
  pAxis = static_cast<ethercatmcIndexerAxis*>(asynMotorController::getAxis(axisNo));
  if (!pAxis) {
    pAxis = new ethercatmcIndexerAxis(this, axisNo, 0, NULL);
  }

  /* In case of re-connect free the old one */
  free(ctrlLocal.pIndexerProcessImage);
  ctrlLocal.pIndexerProcessImage = NULL;
  ctrlLocal.indexerOffset = 4;
  ctrlLocal.firstDeviceStartOffset = 0;
  ctrlLocal.lastDeviceEndOffset = 0;
  memset(&descVersAuthors, 0, sizeof(descVersAuthors));
  if (!ctrlLocal.adsport) {
    ctrlLocal.adsport = 851;
  }
#if 0
  {
    /* Demo only */
    const char *symbolName = "Main.sVersion";
    uint32_t MainsVersionHandle = 0;
    status = getSymbolHandleByNameViaADS(symbolName, &MainsVersionHandle);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s(%s) MainsVersionHandle=0x%X status=%s (%d)\n",
              modNamEMC, symbolName, MainsVersionHandle,
              ethercatmcstrStatus(status), (int)status);
  }
  {
    /* Demo only */
    const char *symbolName = "Main.sVersion";
    AdsSymbolInfoType adsSymbolInfo;
    //uint8_t symbolInfo[1024];
    status = getSymbolInfoViaADS(symbolName,
                                 &adsSymbolInfo, sizeof(adsSymbolInfo));
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s(%s) indexGroup=0x%X indexOffset=0x%X size=%u dataType=%u"
              " flags=0x%X nameLength=%u typeLength=%u commentLength=%u "
              "status=%s (%d)\n",
              modNamEMC, symbolName,
              NETTOUINT(adsSymbolInfo.indexGroup),
              NETTOUINT(adsSymbolInfo.indexOffset),
              NETTOUINT(adsSymbolInfo.size),
              NETTOUINT(adsSymbolInfo.dataType),
              NETTOUINT(adsSymbolInfo.flags),
              NETTOUINT(adsSymbolInfo.nameLength),
              NETTOUINT(adsSymbolInfo.typeLength),
              NETTOUINT(adsSymbolInfo.commentLength),
              ethercatmcstrStatus(status), (int)status);
  }
#endif
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

  status = getPlcMemoryUint(ctrlLocal.indexerOffset,
                            &ctrlLocal.indexerOffset, 2);
  if (status) goto endPollIndexer;
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sindexerOffset=%u\n",
            modNamEMC, ctrlLocal.indexerOffset);

  for (devNum = 0; devNum < 100; devNum++) {
    unsigned iTypCode = -1;
    unsigned iSizeBytes = -1;
    unsigned iOffsBytes = -1;
    unsigned iUnit = -1;
    unsigned iAllFlags = -1;
    double fAbsMin = 0;
    double fAbsMax = 0;
    status = readDeviceIndexer(devNum, infoType0);
    if (status) goto endPollIndexer;
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
    status = getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset +  1*2,
                                            &infoType0_data, sizeof(infoType0_data));
    if (status) goto endPollIndexer;
    iTypCode  = infoType0_data.typCode_0 + (infoType0_data.typCode_1 << 8);
    iSizeBytes= infoType0_data.size_0 + (infoType0_data.size_1 << 8);
    iOffsBytes= infoType0_data.offset_0 + (infoType0_data.offset_1 << 8);
    iUnit     = infoType0_data.unit_0 + (infoType0_data.unit_1 << 8);
    iAllFlags = infoType0_data.flags_0 + (infoType0_data.flags_1 << 8) +
      (infoType0_data.flags_2 << 16) + (infoType0_data.flags_3 << 24);
    fAbsMin   = netToDouble(&infoType0_data.absMin,
                            sizeof(infoType0_data.absMin));
    fAbsMax   = netToDouble(&infoType0_data.absMax,
                            sizeof(infoType0_data.absMax));

    status = readDeviceIndexer(devNum, infoType4);
    if (status) goto endPollIndexer;
    getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset + 1*2,
                                   descVersAuthors.desc,
                                   sizeof(descVersAuthors.desc));
    status = readDeviceIndexer(devNum, infoType5);
    if (status) goto endPollIndexer;
    getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset + 1*2,
                                   descVersAuthors.vers,
                                   sizeof(descVersAuthors.vers));
    status = readDeviceIndexer(devNum, infoType6);
    if (status) goto endPollIndexer;
    getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset + 1*2,
                                   descVersAuthors.author1,
                                   sizeof(descVersAuthors.author1));
    status = readDeviceIndexer(devNum, infoType7);
    if (status) goto endPollIndexer;
    getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset + 1*2,
                                   descVersAuthors.author2,
                                   sizeof(descVersAuthors.author2));
    switch (iTypCode) {
      case 0x5008:
      case 0x500C:
      case 0x5010:
        {
          axisNo++;
        }
      default:
        ;
    }
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%sPilsDevice axisNo=%i devNumPILS=%d \"%s\" TypCode=0x%X OffsBytes=%u "
              "SizeBytes=%u UnitCode=0x%X (%s%s) AllFlags=0x%X AbsMin=%e AbsMax=%e\n",
              modNamEMC, axisNo, devNum, descVersAuthors.desc, iTypCode, iOffsBytes,
              iSizeBytes, iUnit,
              plcUnitPrefixTxt(( (int8_t)((iUnit & 0xFF00)>>8))),
              plcUnitTxtFromUnitCode(iUnit & 0xFF),
              iAllFlags, fAbsMin, fAbsMax);

    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%sdescVersAuthors(%d)  vers=%s author1=%s author2=%s\n",
              modNamEMC, devNum,
              descVersAuthors.vers,
              descVersAuthors.author1,
              descVersAuthors.author2);
    if (!iTypCode && !iSizeBytes && !iOffsBytes) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sfirstDeviceStartOffset=%u lastDeviceEndOffset=%u\n",
                modNamEMC,
                firstDeviceStartOffset,
                lastDeviceEndOffset);
      /* Create memory to keep the processimage for all devices
         We include the non-used bytes at the beginning,
         including the nytes used by the indexer (and waste some bytes)
         to make it easier to understand the adressing using offset */
      /* create a new one, with the right size */

      ctrlLocal.firstDeviceStartOffset = firstDeviceStartOffset;
      ctrlLocal.lastDeviceEndOffset = lastDeviceEndOffset;
      ctrlLocal.pIndexerProcessImage = (uint8_t*)calloc(1, ctrlLocal.lastDeviceEndOffset);
      break; /* End of list */
    }
    /* indexer has devNum == 0, it is not a device */
    if (devNum) {
      unsigned endOffset = iOffsBytes + iSizeBytes;
      if (iOffsBytes < lastDeviceEndOffset) {
        /* There is an overlap */
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sOverlap iOffsBytes=%u lastDeviceEndOffset=%u\n",
                  modNamEMC, iOffsBytes, lastDeviceEndOffset);
        status = asynError;
        goto endPollIndexer;
      }
      /* find the lowest and highest offset for all devices */
      if (iOffsBytes < firstDeviceStartOffset) {
        firstDeviceStartOffset = iOffsBytes;
      }
      if (endOffset > lastDeviceEndOffset) {
        lastDeviceEndOffset = endOffset;
      }
    } else {
#ifdef motorMessageTextString
      /* We find the name of the MCU here */
      setStringParam(0, motorMessageText_, descVersAuthors.desc);
#endif
    }
    switch (iTypCode) {
    case 0x5008:
    case 0x500C:
    case 0x5010:
      {
        char unitCodeTxt[40];
        pAxis = static_cast<ethercatmcIndexerAxis*>(asynMotorController::getAxis(axisNo));
        if (!pAxis) {
          pAxis = new ethercatmcIndexerAxis(this, axisNo, 0, NULL);
        }
        /* Now we have an axis */

        status = newIndexerAxis(pAxis,
                                devNum,
                                iAllFlags,
                                fAbsMin,
                                fAbsMax,
                                iOffsBytes);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sTypeCode axisNo=%d iTypCode=%x pAxis=%p status=%s (%d)\n",
                  modNamEMC, axisNo, iTypCode, pAxis,
                  ethercatmcstrStatus(status), (int)status);
        if (status) goto endPollIndexer;

        pAxis->setIndexerDevNumOffsetTypeCode(devNum, iOffsBytes, iTypCode);
        setStringParam(axisNo,  ethercatmcCfgDESC_RB_, descVersAuthors.desc);
        snprintf(unitCodeTxt, sizeof(unitCodeTxt), "%s%s",
                 plcUnitPrefixTxt(( (int8_t)((iUnit & 0xFF00)>>8))),
                 plcUnitTxtFromUnitCode(iUnit & 0xFF));
        setStringParam(axisNo,  ethercatmcCfgEGU_RB_, unitCodeTxt);
      }
      break;
    case 0x0518:
      /*
       * Total length should be on a 64 bit border
       * We need at least 40 bytes payload -> round up to 48
       * 0x18 == 24dec words -> 48 bytes -> 2byte CTRL + 46 payload */
      if (!strcmp(descVersAuthors.desc, "DbgStrToMcu"))
      {
        ctrlLocal.specialDbgStrToMcuDeviceOffset = iOffsBytes;
        /* Length of the data area in words is "low byte" */
        ctrlLocal.specialDbgStrToMcuDeviceLength = 2 * (iTypCode & 0xFF);
      }
      break;
    default:
      {
        const char *paramName = descVersAuthors.desc;
        int function;

        function = newPilsAsynDevice(axisNo, iOffsBytes, iTypCode, paramName);
        if (function > 0) {
#ifdef ETHERCATMC_ASYN_PARAMMETA
          char metaValue[256];
          memset(&metaValue, 0, sizeof(metaValue));
          if (!strcmp(paramName, "errorID")) {
            snprintf(&metaValue[0], sizeof(metaValue) - 1, "%s",
                     "TwinCAT_errorID");
          } else {
            snprintf(&metaValue[0], sizeof(metaValue) - 1, "%s%s",
                     plcUnitPrefixTxt(( (int8_t)((iUnit & 0xFF00)>>8))),
                     plcUnitTxtFromUnitCode(iUnit & 0xFF));
          }
          if (strlen(metaValue)) {
            asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                      "%s%s(%d) asynParamMeta metaValue=\"%s\")\n",
                      modNamEMC, "initialPoll", axisNo, metaValue);
            setParamMeta(axisNo, function, "EGU", metaValue);
          }
#endif
        }
      }
    }
  }

 endPollIndexer:
  if (status) {
    /* Clean up, if we manged to come half-through only */
    indexerDisconnected();
  }
  return status;

}


int ethercatmcController::addPilsAsynDevLst(int           axisNo,
                                            const char    *paramName,
                                            unsigned      lenInPLC,
                                            unsigned      inputOffset,
                                            unsigned      outputOffset,
                                            unsigned      statusOffset,
                                            asynParamType myEPICSParamType,
                                            unsigned      iTypCode)
{
  const static char *const functionName = "addPilsAsynDevLst";
  unsigned numPilsAsynDevInfo = ctrlLocal.numPilsAsynDevInfo;
  static size_t maxNumPilsAsynDevInfo =
    (sizeof(ctrlLocal.pilsAsynDevInfo) / sizeof(ctrlLocal.pilsAsynDevInfo[0])) - 1;

  asynStatus status;
  int function = -1;
  pilsAsynDevInfo_type *pPilsAsynDevInfo
    = &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

  if (numPilsAsynDevInfo >= maxNumPilsAsynDevInfo) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
              "%s%s(%u) out of range\n",
              modNamEMC, functionName,  numPilsAsynDevInfo);
    return -1;
  }

  asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
            "%s%s axisNo=%i \"%s\" lenInPLC=%u inputOffset=%u outputOffset=%u statusOffset=%u"
            " EPICSParamType=%s(%i) iTypeCode=0x%04X\n",
            modNamEMC, functionName, axisNo,
            paramName,
            lenInPLC,
            inputOffset,
            outputOffset,
            statusOffset,
            stringFromAsynParamType(myEPICSParamType), (int)myEPICSParamType,
            iTypCode);
  if (!strcmp(paramName, "encoderRaw")) {
      paramName = "EncAct";
      /* Special handling for encoderRaw */
      myEPICSParamType = asynParamFloat64;
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s(%u) \"%s\" EPICSParamType=%s(%i)\n",
            modNamEMC, functionName, axisNo,
            paramName,
            stringFromAsynParamType(myEPICSParamType),
            (int)myEPICSParamType);

  /* Some parameters are alread pre-created by the Controller.cpp,
     e.g.errorId. Use those, otherwise create a parameter */
  status = findParam(/* axisNo, */paramName, &function);
  if (status == asynSuccess) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s exist function=%d paramName=%s\n",
              modNamEMC, functionName, function, paramName);
  } else {
    status = createParam(/* axisNo, */
                         paramName,
                         myEPICSParamType,
                         &function);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s(%u) numPilsAsynDevInfo=%d created function=%d paramName=%s status=%s (%d)\n",
              modNamEMC, functionName, axisNo, numPilsAsynDevInfo, function,
              paramName,
              ethercatmcstrStatus(status), (int)status);
    if (status != asynSuccess) return -1;
  }

  pPilsAsynDevInfo->axisNo           = axisNo;
  pPilsAsynDevInfo->lenInPLC         = lenInPLC;
  pPilsAsynDevInfo->inputOffset      = inputOffset;
  pPilsAsynDevInfo->outputOffset     = outputOffset;
  pPilsAsynDevInfo->statusOffset     = statusOffset;
  pPilsAsynDevInfo->myEPICSParamType = myEPICSParamType;
  pPilsAsynDevInfo->iTypCode         = iTypCode;
  pPilsAsynDevInfo->function         = function;
  if (!strcmp(paramName, "SystemUTCtime")) {
    pPilsAsynDevInfo->isSystemUTCtime = 1;
    ctrlLocal.systemUTCtimeOffset = inputOffset;
  }
  setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);

  /* Last action of this code: Increment the counter */
  ctrlLocal.numPilsAsynDevInfo = 1 + numPilsAsynDevInfo;
  return function;
}


int ethercatmcController::newPilsAsynDevice(int      axisNo,
                                            unsigned indexOffset,
                                            unsigned iTypCode,
                                            const char *paramName)
{
  const static char *const functionName = "newPilsAsynDevice";
  unsigned numPilsAsynDevInfo = ctrlLocal.numPilsAsynDevInfo;
  unsigned      lenInPLC          = 0;
  unsigned      inputOffset       = 0;
  unsigned      outputOffset      = 0;
  unsigned      statusOffset = 0;
  asynParamType myAsynParamType = asynParamNotDefined;
  struct {
    char     name[80];      /* 34 + some spare */
    unsigned axisNoOrIndex;
  } splitedParamNameNumber;

  if (strlen(paramName) < sizeof(splitedParamNameNumber.name)) {
    /* Need to split the parameter, like "EPOCHEL1252P#1" */
    int nvals;
    memset(&splitedParamNameNumber, 0, sizeof(splitedParamNameNumber));
    nvals = sscanf(paramName, "%[^#]#%u",
                   &splitedParamNameNumber.name[0],
                   &splitedParamNameNumber.axisNoOrIndex);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s nvals=%d name=\"%s\" axisNoOrIndex=%u\n",
              modNamEMC, functionName, nvals,
              &splitedParamNameNumber.name[0],
              splitedParamNameNumber.axisNoOrIndex);
    if (nvals == 2) {
      paramName = &splitedParamNameNumber.name[0];
      axisNo = (int)splitedParamNameNumber.axisNoOrIndex;
    }
  }

  if (!iTypCode) return -1;
  switch (iTypCode) {
    case 0x1201:
      lenInPLC = 2;
      inputOffset = indexOffset;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1202:
      lenInPLC = 4;
      inputOffset = indexOffset;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1204:
      lenInPLC = 8;
      inputOffset = indexOffset;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
      myAsynParamType = asynParamInt64;
#endif
      break;
    case 0x1302:
      lenInPLC = 4;
      inputOffset = indexOffset;
      myAsynParamType = asynParamFloat64;
      break;
    case 0x1304:
      lenInPLC = 8;
      inputOffset = indexOffset;
      myAsynParamType = asynParamFloat64;
      break;
    case 0x1602:
      lenInPLC = 2;
      /* 1602 has "current value, followed by target value */
      inputOffset = indexOffset + lenInPLC; // Look at the target value for readback
      outputOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1604:
      lenInPLC = 4;
      /* 1604 has "current value, followed by target value */
      inputOffset = indexOffset + lenInPLC;  // Look at the target value for readback
      outputOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1A02:
      lenInPLC = 2;
      /* 1A02 has "current value, followed by status word */
      inputOffset = indexOffset;
      statusOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1A04:
      lenInPLC = 4;
      /* 1A04 has "current value, followed by extended status word */
      inputOffset = indexOffset;
      statusOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
  }
  if (!lenInPLC) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s(%u) iTypCode not supported, ignored 0x%X\n",
              modNamEMC, functionName, numPilsAsynDevInfo, iTypCode);
    return -1;
  }
  if (myAsynParamType != asynParamNotDefined) {
    return addPilsAsynDevLst(axisNo,
                             paramName,
                             lenInPLC,
                             inputOffset,
                             outputOffset,
                             statusOffset,
                             myAsynParamType,
                             iTypCode);
  } else {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s(%u) pilsNo=%d not created paramName=%s)\n",
              modNamEMC, functionName, axisNo, numPilsAsynDevInfo, paramName);
  }
  return -1;
}

extern "C" void UTCtimeToEpicsTimeStamp(uint64_t dcNsec, epicsTimeStamp *ts)
{
#define NSEC_PER_SEC      1000000000
  /*
   * convert from UTC Time to Epics time
   * (POSIX_TIME_AT_EPICS_EPOCH defined in epicsTime.h)
  */
  uint64_t nSecEpicsEpoch;
  nSecEpicsEpoch = dcNsec - ((uint64_t)POSIX_TIME_AT_EPICS_EPOCH) * NSEC_PER_SEC;
  ts->secPastEpoch = (uint32_t)(nSecEpicsEpoch / NSEC_PER_SEC);
  ts->nsec =         (uint32_t)(nSecEpicsEpoch % NSEC_PER_SEC);
}

/*
 * Find if an asyn parameter is connected to an indexer output
 */
pilsAsynDevInfo_type *ethercatmcController::findIndexerOutputDevice(int axisNo,
                                                                    int function,
                                                                    asynParamType myEPICSParamType)
{
  for (unsigned numPilsAsynDevInfo = 0;
       numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
       numPilsAsynDevInfo++) {
    pilsAsynDevInfo_type *pPilsAsynDevInfo
      = &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

    if ((axisNo == pPilsAsynDevInfo->axisNo) &&
        (function == pPilsAsynDevInfo->function) &&
        (myEPICSParamType == pPilsAsynDevInfo->myEPICSParamType) &&
        (pPilsAsynDevInfo->outputOffset))
      return pPilsAsynDevInfo;
  }
  return NULL;
}

asynStatus ethercatmcController::indexerPoll(void)
{
  int callBacksNeeded = 0;
  if (ctrlLocal.pIndexerProcessImage &&
      ctrlLocal.lastDeviceEndOffset) {
    size_t indexOffset = ctrlLocal.firstDeviceStartOffset;
    size_t len = ctrlLocal.lastDeviceEndOffset - indexOffset;
    asynStatus status;
    int traceMask = ASYN_TRACEIO_DRIVER;
    memset(ctrlLocal.pIndexerProcessImage, 0,
           ctrlLocal.lastDeviceEndOffset);
    status = getPlcMemoryOnErrorStateChange(indexOffset,
                                            &ctrlLocal.pIndexerProcessImage[indexOffset],
                                            len);
    if (status) traceMask |= ASYN_TRACE_ERROR;
    asynPrint(pasynUserController_, traceMask,
              "%spoll() indexOffset=%u len=%u status=%s (%d)\n",
              modNamEMC, (unsigned)indexOffset, (unsigned)len,
              ethercatmcstrStatus(status), (int)status);
    if (status) return status;

    {
      /* Extract devices, which are not motors */
      for (unsigned numPilsAsynDevInfo = 0;
           numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
           numPilsAsynDevInfo++) {
        pilsAsynDevInfo_type *pPilsAsynDevInfo
          = &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

        unsigned inputOffset = pPilsAsynDevInfo->inputOffset;
        unsigned statusOffset = pPilsAsynDevInfo->statusOffset;
        int axisNo = pPilsAsynDevInfo->axisNo;
        int function = pPilsAsynDevInfo->function;
        void *pDataInPlc = &ctrlLocal.pIndexerProcessImage[inputOffset];
        asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                  "%sindexerPoll(%d) numPilsAsynDevInfo=%u inputOffset=%u\n",
                  modNamEMC, axisNo,
                  numPilsAsynDevInfo, inputOffset);
        if (statusOffset) {
          ; /* TODO */
        }
        if (!inputOffset) continue;


        /* Each axis has it's own parameters.
           axisNo == 0 is no special axis, parameters
           for "the controller", additional IO, PTP info */
        switch (pPilsAsynDevInfo->myEPICSParamType) {
        case asynParamInt32:
          {
            int tracelevel = ASYN_TRACE_FLOW;
            const char *paramName = "";
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            epicsInt32 newValue, oldValue;
            getParamName(axisNo, function, &paramName);
            newValue = (epicsInt32)netToSint(pDataInPlc, lenInPLC);
            status = getIntegerParam(axisNo, function, &oldValue);
            if (status != asynSuccess || oldValue != newValue) {
              if (pPilsAsynDevInfo->outputOffset) tracelevel |= ASYN_TRACE_INFO;
              status = setIntegerParam(axisNo, function,  newValue);
              if (status == asynParamWrongType) {
                asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                          "%sindexerPoll(%d) ERROR: need to disable function=%s(%d) status=%s (%d)\n",
                          modNamEMC, axisNo,
                          paramName, function,
                          ethercatmcstrStatus(status), (int)status);
                pPilsAsynDevInfo->inputOffset = 0;
              } else {
                callBacksNeeded = 1;
                if (status != asynSuccess)  tracelevel |= ASYN_TRACE_INFO;
              }
            }
            asynPrint(pasynUserController_, tracelevel,
                      "%sindexerPoll(%d) function=%s(%d) oldValue=%d newValue=%d\n",
                      modNamEMC, axisNo,
                      paramName, function, oldValue, newValue);
          }
          break;
        case asynParamFloat64:
          /* Float64 may be read from integers or float or double */
          {
            int tracelevel = ASYN_TRACE_FLOW;
            const char *paramName = "";
            double newValue = 0.0, oldValue = 0.0;
            int newValueValid = 1;
            unsigned iTypCode = pPilsAsynDevInfo->iTypCode;
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            switch (iTypCode) {
            case 0x1302:
            case 0x1304:
              newValue = (double)netToDouble(pDataInPlc, lenInPLC);
              break;
            case 0x1201:
            case 0x1202:
            case 0x1602:
            case 0x1604:
            case 0x1A02:
            case 0x1A04:
              newValue = (double)(epicsInt64)netToSint(pDataInPlc, lenInPLC);
              break;
            case 0x1204:
            case 0x1A08:
              newValue = (double)(epicsInt64)netToSint64(pDataInPlc, lenInPLC);
              break;
            default:
              asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                        "%sindexerPoll(%d) ERROR: newValueValid = 0 function=%s(%d)\n",
                        modNamEMC, axisNo,
                        paramName, function);
              newValueValid = 0;
            }
            getParamName(axisNo, function, &paramName);
            if (newValueValid) {
              status = getDoubleParam(axisNo, function,  &oldValue);
              if (status != asynSuccess || oldValue != newValue) {
                status = setDoubleParam(axisNo, function,  newValue);
                if (status == asynParamWrongType) {
                  asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                            "%sindexerPoll(%d) ERROR: need to disable function=%s(%d) status=%s (%d)\n",
                            modNamEMC, axisNo,
                            paramName, function,
                            ethercatmcstrStatus(status), (int)status);
                  pPilsAsynDevInfo->inputOffset = 0;
                } else {
                  callBacksNeeded = 1;
                  if (status != asynSuccess)  tracelevel |= ASYN_TRACE_INFO;
                }
              }
            }
            asynPrint(pasynUserController_, tracelevel,
                      "%sindexerPoll(%d) function=%s(%d) oldValue=%f newValue=%f\n",
                      modNamEMC, axisNo,
                      paramName, function, oldValue, newValue);
          }
          break;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
        case asynParamInt64:
          {
            int tracelevel = ASYN_TRACE_FLOW;
            const char *paramName = "";
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            epicsInt64 newValue, oldValue;
            getParamName(axisNo, function, &paramName);
            newValue = (epicsInt64)netToSint64(pDataInPlc, lenInPLC);
            status = getInteger64Param(axisNo, function,  &oldValue);
            if (status != asynSuccess || oldValue != newValue) {
              status = setInteger64Param(axisNo, function,  newValue);
              if (status == asynParamWrongType) {
                asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                          "%sindexerPoll(%d) ERROR: need to disable function=%s(%d) status=%s (%d)\n",
                          modNamEMC, axisNo,
                          paramName, function,
                          ethercatmcstrStatus(status), (int)status);
                pPilsAsynDevInfo->inputOffset = 0;
              } else {
                callBacksNeeded = 1;
                if (status != asynSuccess)  tracelevel |= ASYN_TRACE_INFO;
              }
            }
            asynPrint(pasynUserController_, tracelevel,
                      "%sindexerPoll(%d) function=%s(%d)  oldValue=%" PRIi64 " newValue=%" PRIi64 "\n",
                      modNamEMC, axisNo,
                      paramName, function, (int64_t)oldValue, (int64_t)newValue);
          }
          break;
#endif
        default:
          ;
        }
        if (pPilsAsynDevInfo->isSystemUTCtime) {
          uint64_t nSec;
          epicsTimeStamp timeStamp;
          unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
          nSec = netToUint64(pDataInPlc, lenInPLC);
          UTCtimeToEpicsTimeStamp(nSec, &timeStamp);
          asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
                    "%sindexerPoll SystemUTCtime nSec=%" PRIu64 " sec:nSec=%09u.%09u\n",
                    modNamEMC, nSec,
                    timeStamp.secPastEpoch, timeStamp.nsec);
          setTimeStamp(&timeStamp);
          callBacksNeeded = 1;
        }
      } /* for */
    }
    if (callBacksNeeded) {
      callParamCallbacks();
    }
    return status;
  }
  return asynDisabled;
}


void ethercatmcController::indexerDisconnected(void)
{
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s\n",
            modNamEMC, "indexerDisconnected");
  for (int axisNo=0; axisNo<numAxes_; axisNo++) {
    setIntegerParam(axisNo, motorStatusGainSupport_, 0);
    for (int function = ethercatmcNamAux0_;
         function < ethercatmcNamAux0_ + MAX_AUX_BIT_SHOWN;
         function++) {
      setAlarmStatusSeverityWrapper(axisNo, function, asynDisconnected);
    }
  }

  if (ctrlLocal.numPilsAsynDevInfo)
  {
    for (unsigned numPilsAsynDevInfo = 0;
         numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
         numPilsAsynDevInfo++) {
      pilsAsynDevInfo_type *pPilsAsynDevInfo
        = &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];
      setAlarmStatusSeverityWrapper(pPilsAsynDevInfo->axisNo,
                                    pPilsAsynDevInfo->function,
                                    asynDisconnected);
    }
    memset(&ctrlLocal.pilsAsynDevInfo, 0, sizeof(ctrlLocal.pilsAsynDevInfo));
    ctrlLocal.numPilsAsynDevInfo = 0;
  }
  free(ctrlLocal.pIndexerProcessImage);
  ctrlLocal.pIndexerProcessImage = NULL;
}

