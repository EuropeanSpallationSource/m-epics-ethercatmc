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
  int paramIndexIsIntegerV2(unsigned paramIndex)
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

asynStatus ethercatmcController::indexerParamReadFL(ethercatmcIndexerAxis *pAxis,
                                                    unsigned paramIfOffset,
                                                    unsigned paramIndex,
                                                    double   *value,
                                                    const char *fileName,
                                                    int lineNo)
{
  paramIf_type paramIf_from_MCU;
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned cmd      = PARAM_IF_CMD_DOREAD + paramIndex;
  unsigned counter = 0;
  unsigned lenInPlcPara = 0;
  if (pAxis->drvlocal.lenInPlcParaInteger[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.lenInPlcParaInteger[paramIndex];
  } else if (pAxis->drvlocal.lenInPlcParaFloat[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.lenInPlcParaFloat[paramIndex];
  }
  if (!paramIfOffset || paramIndex > 0xFF || !lenInPlcPara ||
      lenInPlcPara > sizeof(paramIf_from_MCU.paramValueRaw)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s paramIndex=%u lenInPlcPara=%u paramIfOffset=%u\n",
              modNamEMC, paramIndex, lenInPlcPara, paramIfOffset);
    return asynDisabled;
  }
  size_t lenInPLCparamIf = sizeof(paramIf_from_MCU.paramCtrl) + lenInPlcPara;
  while (counter < MAX_COUNTER) {
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    status = getPlcMemoryOnErrorStateChange(paramIfOffset,
                                            &paramIf_from_MCU,
                                            lenInPLCparamIf);

    if (status) {
      int axisNo = pAxis->axisNo_;
      asynPrint(pasynUserController_, traceMask | ASYN_TRACE_INFO,
                "%s:%d %s(%d) paramIfOffset=%u lenInPLCparamIf=%u status=%s (%d)\n",
                fileName, lineNo, "indexerParamRead", axisNo,
                paramIfOffset, (unsigned)lenInPLCparamIf,
                ethercatmcstrStatus(status), (int)status);
      return status;
    }
    unsigned cmdSubParamIndexRB = NETTOUINT(paramIf_from_MCU.paramCtrl);
    unsigned paramIndexRB = cmdSubParamIndexRB & PARAM_IF_IDX_MASK;
    if (counter > 1) {
      int axisNo = pAxis->axisNo_;
      traceMask |= ASYN_TRACE_INFO;
      asynPrint(pasynUserController_, traceMask, /* | ASYN_TRACE_INFO, */
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
        double fValue = -1; //NaN;
        if (pAxis->drvlocal.lenInPlcParaInteger[paramIndex]) {
          fValue = (double)netToUint(&paramIf_from_MCU.paramValueRaw,
                                     lenInPlcPara);
        } else if (pAxis->drvlocal.lenInPlcParaFloat[paramIndex]) {
          fValue = netToDouble(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
        }
        asynPrint(pasynUserController_, traceMask /* | ASYN_TRACE_INFO */,
                  "%s:%d %s(%d) paramIfOffset=%u paramIdxFunction=%s (%u 0x%02X) "
                  "lenInPlcParaFloat=%u lenInPlcParaInteger=%u lenInPlcPara=%u value=%f\n",
                  fileName, lineNo, "indexerParamRead", pAxis->axisNo_, paramIfOffset,
                  plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                  pAxis->drvlocal.lenInPlcParaFloat[paramIndex],
                  pAxis->drvlocal.lenInPlcParaInteger[paramIndex],
                  lenInPlcPara, fValue);

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
        int axisNo = pAxis->axisNo_;
        status = asynDisabled;
        asynPrint(pasynUserController_, traceMask | ASYN_TRACE_INFO,
                  "%s:%d %s(%d) paramIfOffset=%u paramIdxFunction=%s (%u 0x%02X) "
                  "cmdSubParamIndexRB=%s (0x%04X) status=%s (%d)\n",
                  fileName, lineNo, "indexerParamRead", axisNo, paramIfOffset,
                  plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                  paramIfCmdToString(cmdSubParamIndexRB), cmdSubParamIndexRB,
                  ethercatmcstrStatus(status), (int)status);
        return status;
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

asynStatus ethercatmcController::indexerParamWrite(ethercatmcIndexerAxis *pAxis,
                                                   unsigned paramIndex,
                                                   double value,
                                                   double *pValueRB)
{
  int axisNo = pAxis->axisNo_;
  paramIf_type paramIf_to_MCU;
  paramIf_type paramIf_from_MCU;
  unsigned traceMask = ASYN_TRACE_INFO;
  asynStatus status = asynSuccess;
  unsigned cmd      = PARAM_IF_CMD_DOWRITE + paramIndex;
  unsigned counter = 0;
  int has_written = 0;
  unsigned lenInPlcPara = 0;
  unsigned paramIfOffset = pAxis->drvlocal.paramIfOffset;

  if (pAxis->drvlocal.lenInPlcParaInteger[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.lenInPlcParaInteger[paramIndex];
  } else if (pAxis->drvlocal.lenInPlcParaFloat[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.lenInPlcParaFloat[paramIndex];
  }
  if (!pAxis || !paramIfOffset || (paramIndex > 0xFF) ||
      lenInPlcPara > sizeof(paramIf_to_MCU.paramValueRaw)) {
    status = asynDisabled;
  } else if (pAxis->drvlocal.PILSparamPerm[paramIndex] == PILSparamPermRead) {
    status = asynParamWrongType;
  } else if (pAxis->drvlocal.PILSparamPerm[paramIndex] == PILSparamPermNone) {
    status = asynParamBadIndex;
  }
  if (status != asynSuccess) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s pAxis=%p paramIndex=%u lenInPlcPara=%u paramIfOffset=%u perm=%d status=%s (%d)\n",
              modNamEMC, pAxis, paramIndex, lenInPlcPara, paramIfOffset,
              (int)pAxis->drvlocal.PILSparamPerm[paramIndex],
              ethercatmcstrStatus(status), (int)status);
    return status;
  }
  size_t lenInPLCparamIf = sizeof(paramIf_to_MCU.paramCtrl) + lenInPlcPara;

  memset(&paramIf_to_MCU, 0, sizeof(paramIf_to_MCU));
  memset(&paramIf_from_MCU, 0, sizeof(paramIf_from_MCU));

  if (pAxis->drvlocal.lenInPlcParaInteger[paramIndex]) {
    uintToNet((int)value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  } else if (pAxis->drvlocal.lenInPlcParaFloat[paramIndex]) {
    doubleToNet(value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  }
  UINTTONET(cmd, paramIf_to_MCU.paramCtrl);

  while (counter < MAX_COUNTER) {
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    status = getPlcMemoryOnErrorStateChange(paramIfOffset,
                                            &paramIf_from_MCU,
                                            lenInPLCparamIf);
    if (status) return status;
    double valueRB = -1.0;
    if (pAxis->drvlocal.lenInPlcParaInteger[paramIndex]) {
      valueRB = netToUint(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    } else if (pAxis->drvlocal.lenInPlcParaFloat[paramIndex]) {
      valueRB = netToDouble(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    }
    unsigned cmdSubParamIndexRB = NETTOUINT(paramIf_from_MCU.paramCtrl);
    unsigned paramIndexRB = cmdSubParamIndexRB & PARAM_IF_IDX_MASK;

    if (counter >= 1) {
      asynPrint(pasynUserController_, traceMask,
                "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g "
                "counter=%u RB=%s (0x%04X)\n",
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
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g valueRB=%02g has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
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
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g lenInPlcPara=%u has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                    value, lenInPlcPara, has_written);
          status = setPlcMemoryOnErrorStateChange(paramIfOffset, &paramIf_to_MCU,
                                                  (unsigned)sizeof(paramIf_to_MCU));
          if (status) return status;
          has_written = 1;
        } else if (paramIndexRB == paramIndex) {
          status = asynDisabled;
          if (pAxis) {
            if (paramIfCmd == PARAM_IF_CMD_ERR_NO_IDX) {
              status = asynParamBadIndex;
            } else if (paramIfCmd == PARAM_IF_CMD_ERR_READONLY) {
              if (ctrlLocal.supported.bPILSv2) {
                // When PILS V2 "announces" a parameter, there is no
                //   destinction between "read" and "write"
                //   Change the permissions here
                pAxis->drvlocal.PILSparamPerm[paramIndex] = PILSparamPermRead;
              }
              status = asynParamWrongType;
            }
          }
        }
        if (status != asynSuccess) {
          asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                    "%s pAxis=%p paramIndex=%u lenInPlcPara=%u paramIfOffset=%u status=%s (%d)\n",
                    modNamEMC, pAxis, paramIndex, lenInPlcPara, paramIfOffset,
                    ethercatmcstrStatus(status), (int)status);
          return status;
        }
      }
      break;
    case PARAM_IF_CMD_BUSY:
      {
        /* A "function" goes into busy - and stays there */
        /* No parameter settings during jogging/homing */
        if (paramIndexIsMovingFunction(paramIndex)) {
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g movingFun RB=%s (0x%04X)\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex, paramIndex,
                    value,
                    paramIfCmdToString(cmdSubParamIndexRB), cmdSubParamIndexRB);
          if (paramIndexRB == paramIndex) {
            /* "our" function: return */
            if (pValueRB) *pValueRB = valueRB;
            return asynSuccess;
          }
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

int ethercatmcController::paramIndexToFunction(unsigned paramIndex)
{
  switch(paramIndex) {
  case PARAM_IDX_OPMODE_AUTO_UINT:
    /* CNEN for EPICS */
    return motorStatusGainSupport_;
  case PARAM_IDX_MICROSTEPS_FLOAT:
    return ethercatmcCfgSREV_RB_;
  case PARAM_IDX_USR_MIN_FLOAT:
    return ethercatmcCfgDLLM_RB_;
  case PARAM_IDX_ABS_MIN_FLOAT:
    return ethercatmcCfgPMIN_RB_;
  case PARAM_IDX_ABS_MAX_FLOAT:
   return ethercatmcCfgPMAX_RB_;
  case PARAM_IDX_USR_MAX_FLOAT:
    return ethercatmcCfgDHLM_RB_;
  //case PARAM_IDX_WRN_MIN_FLOAT:
  //case PARAM_IDX_WRN_MAX_FLOAT:
  case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
    return ethercatmcCfgPOSLAG_RB_;
  case PARAM_IDX_HYTERESIS_FLOAT:
    return ethercatmcCfgSPDB_RB_;
    //return ethercatmcCfgRDBD_RB_;
  case PARAM_IDX_REFSPEED_FLOAT:
    return ethercatmcCfgHVEL_RB_;
  case PARAM_IDX_SPEED_FLOAT:
    return ethercatmcVel_RB_;
  case PARAM_IDX_ACCEL_FLOAT:
    return ethercatmcAcc_RB_;
  //case PARAM_IDX_IDLE_CURRENT_FLOAT:
  //case PARAM_IDX_MOVE_CURRENT_FLOAT:
  //case PARAM_IDX_MICROSTEPS_UINT:
  //case PARAM_IDX_STEPS_PER_UNIT_FLOAT:
    return ethercatmcCfgUREV_RB_;
  case PARAM_IDX_HOME_POSITION_FLOAT:
    return ethercatmcHomPos_RB_;
  //case PARAM_IDX_FUN_REFERENCE:
  //case PARAM_IDX_FUN_SET_POSITION:
  case PARAM_IDX_FUN_MOVE_VELOCITY:
    return ethercatmcCfgJVEL_RB_;
  case PARAM_IDX_USR_MIN_EN_UINT:
    return ethercatmcCfgDLLM_En_RB_;
  case PARAM_IDX_USR_MAX_EN_UINT:
    return ethercatmcCfgDHLM_En_RB_;
  case PARAM_IDX_HOMPROC_UINT:
    return ethercatmcHomProc_RB_;
  case PARAM_IDX_USR_MIN_EN_FLOAT:
    return ethercatmcCfgDLLM_En_RB_;
  case PARAM_IDX_USR_MAX_EN_FLOAT:
    return ethercatmcCfgDHLM_En_RB_;
  case PARAM_IDX_HOMPROC_FLOAT:
    return ethercatmcHomProc_RB_;
  case PARAM_IDX_UNITS_PER_REV_FLOAT:
    return ethercatmcCfgUREV_RB_;
  case PARAM_IDX_STEPS_PER_REV_FLOAT:
    return ethercatmcCfgSREV_RB_;
  case PARAM_IDX_MAX_VELO_FLOAT:
    return ethercatmcCfgVMAX_RB_;
  default:
    return 0;
  }
}

void ethercatmcController::parameterFloatReadBack(unsigned axisNo,
                                                  int initial,
                                                  unsigned paramIndex,
                                                  double fValue)
{
  const static double fullsrev = 200;    /* (default) Full steps/revolution */

  switch(paramIndex) {
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
                                                unsigned devNum)
{
  unsigned axisNo = pAxis->axisNo_;
  asynStatus status = asynError;
  /* Find out which parameters that exist for this device
     The result is stored in pAxis->drvlocal.PILSparamPerm[] */

  unsigned paramIfOffset = pAxis->drvlocal.paramIfOffset;
  if (ctrlLocal.supported.bPILSv2) {
    status = indexerReadAxisParametersV2(pAxis, devNum);
  } else if (ctrlLocal.supported.bPILSv3) {
    status = asynSuccess; /* see indexerV3readParameterDescriptors() */
  }
  if (status) return status;
  /* loop through all parameters.
     PILS v2 and v3 use the same param interface logic */
  unsigned paramIndex;

  for (paramIndex = 0; paramIndex < (sizeof(pAxis->drvlocal.PILSparamPerm) /
                                     sizeof(pAxis->drvlocal.PILSparamPerm[0]));
       paramIndex++) {

    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%sindexerReadAxisParameters(%d) paramIdx=%s (%u) perm=%d\n",
              modNamEMC, axisNo,
              plcParamIndexTxtFromParamIndex(paramIndex),
              paramIndex,
              (int)pAxis->drvlocal.PILSparamPerm[paramIndex]);
    if (pAxis->drvlocal.PILSparamPerm[paramIndex] != PILSparamPermNone) {
      // parameter is read or write
      double fValue = 0.0;
      int initial = 1;
      switch(paramIndex) {
      case PARAM_IDX_OPMODE_AUTO_UINT:
        /* CNEN for EPICS */
        pAxis->setIntegerParam(motorStatusGainSupport_, 1);
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
      }
      if (paramIndexIsParameterToPoll(paramIndex) ||
          (paramIndex == PARAM_IDX_FUN_MOVE_VELOCITY)) {
        /* Some parameters are functions: Don't read them.
           tell driver that the function exist
           But read 142, which becomes JVEL */
        if (pAxis->drvlocal.enumparam_read_id[paramIndex]) {
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%s (%u) has enums\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex),
                    paramIndex);
        } else if (paramIndexIsReadLaterInBackground(paramIndex)) {
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%s (%u) only polled in background\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex),
                    paramIndex);
        } else {
          status = indexerParamRead(pAxis,
                                    paramIfOffset,
                                    paramIndex,
                                    &fValue);
          if (status) {
            asynPrint(pasynUserController_,
                      ASYN_TRACE_INFO,
                      "%sindexerReadAxisParameters(%d) paramIdx=%s (%u)"
                      " status=%s (%d)\n",
                      modNamEMC, axisNo,
                      plcParamIndexTxtFromParamIndex(paramIndex),
                      paramIndex,
                      ethercatmcstrStatus(status), (int)status);
            return status;
          }
          parameterFloatReadBack(axisNo, initial, paramIndex, fValue);
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%s (%u) value=%f\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex),
                    paramIndex, fValue);
        }
      }
      if (paramIndexIsParameterToPoll(paramIndex)) {
        pAxis->addPollNowParam(paramIndex);
      }
    }
  }
  return status;
}

asynStatus ethercatmcController::indexerInitialPoll(void)
{
  asynStatus status;
  {
    ethercatmcIndexerAxis *pAxis;
    int axisNo = 0;
    pAxis = static_cast<ethercatmcIndexerAxis*>(asynMotorController::getAxis(axisNo));
    if (!pAxis) {
      pAxis = new ethercatmcIndexerAxis(this, axisNo, 0, NULL);
    }
  }

  /* In case of re-connect free the old one */
  free(ctrlLocal.pIndexerProcessImage);
  ctrlLocal.pIndexerProcessImage = NULL;
  ctrlLocal.indexerOffset = 4;
  ctrlLocal.firstDeviceStartOffset = 0;
  ctrlLocal.lastDeviceEndOffset = 0;
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
  unsigned iTmpVer = 0xC0DEAFFE;
  {
    status = getPlcMemoryUint(0, &iTmpVer, sizeof(iTmpVer));
    if (status) {
#ifdef motorMessageTextString
      if (pasynUserController_) {
        asynUser *pasynUser = pasynUserController_;
        setStringParam(0, motorMessageText_, pasynUser->errorMessage);
      }
#endif
      return status;
    }
  }
  memset(&ctrlLocal.supported, 0, sizeof(ctrlLocal.supported));
  /* Before the version is checked, get the offset, since we it anyway */
  status = getPlcMemoryUint(ctrlLocal.indexerOffset,
                            &ctrlLocal.indexerOffset, 2);
  if (!status) {
    const char *version = "";
    if (iTmpVer == 0x44fbe0a4) {
      ctrlLocal.supported.bPILSv2 = 1;
      version = "2015.02";
      status = indexerInitialPollv2();
    } else if (iTmpVer == 0x44fca2e1) {
      ctrlLocal.supported.bPILSv3 = 1;
      version = "2021.09";
      status = indexerInitialPollv3();
    } else {
      status = asynDisabled;
    }
    if (status == asynSuccess) {
      ctrlLocal.cntADSstatus = 0;
    }
    if (ctrlLocal.cntADSstatus < MAXCNTADSSTATUS) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%sadsport=%u version='%s' 0x%X indexerOffset=%u\n",
                modNamEMC, ctrlLocal.adsport, version, iTmpVer,
                ctrlLocal.indexerOffset);
      ctrlLocal.cntADSstatus++;
    }
    if (status != asynSuccess) {
#ifdef motorMessageTextString
      char buf[64];
      snprintf(buf, sizeof(buf), "IndexerVers 0x%08X", iTmpVer);
      (void)setStringParam(motorMessageText_, buf);
#endif
      /* Clean up, if we manged to come half-through only */
      indexerDisconnected();
    }
  }
  return status;
}

int ethercatmcController::addPilsAsynDevLst(int           axisNo,
                                            const char    *paramName,
                                            int           functionNamAux0,
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
            "%s%s axisNo=%i \"%s\" functionNamAux0=%d lenInPLC=%u inputOffset=%u outputOffset=%u statusOffset=%u"
            " EPICSParamType=%s(%i) iTypeCode=0x%04X\n",
            modNamEMC, functionName, axisNo,
            paramName,
            functionNamAux0,
            lenInPLC,
            inputOffset,
            outputOffset,
            statusOffset,
            stringFromAsynParamType(myEPICSParamType), (int)myEPICSParamType,
            iTypCode);
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
  pPilsAsynDevInfo->functionNamAux0  = functionNamAux0;
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
    // We will calculate the PV in poll()
    setAlarmStatusSeverityWrapper(axisNo, ethercatmcPTPdiffTimeIOC_MCU_,
                                  asynSuccess);
  }
  setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);

  /* Last action of this code: Increment the counter */
  ctrlLocal.numPilsAsynDevInfo = 1 + numPilsAsynDevInfo;
  return function;
}


int ethercatmcController::newPilsAsynDevice(int      axisNo,
                                            unsigned indexOffset,
                                            unsigned iTypCode,
                                            unsigned iAllFlags,
                                            const char *paramName)
{
  const static char *const functionName = "newPilsAsynDevice";
  unsigned numPilsAsynDevInfo = ctrlLocal.numPilsAsynDevInfo;
  unsigned      lenInPLC          = 0;
  unsigned      inputOffset       = 0;
  unsigned      outputOffset      = 0;
  unsigned      statusOffset      = 0;
  int           functionNamAux0   = 0;
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
              "%s%s axisNo=%d iTypCode=0x%04X nvals=%d name=\"%s\" axisNoOrIndex=%u\n",
              modNamEMC, functionName, axisNo, iTypCode, nvals,
              &splitedParamNameNumber.name[0],
              splitedParamNameNumber.axisNoOrIndex);
    if (nvals == 2) {
      int newAxisNo = (int)splitedParamNameNumber.axisNoOrIndex;
      paramName = &splitedParamNameNumber.name[0];
      axisNo = newAxisNo;
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
    case 0x1802:
      {
        unsigned i;
        lenInPLC = 4;
        //paramName = ethercatmcStatusBitsString;
        /* 1802 has only a 32 bit status word */
        statusOffset = indexOffset;
        myAsynParamType = asynParamUInt32Digital;
        if (iAllFlags & 0x03FFFFFF) {
          for (i=0; i < MAX_REASON_AUX_BIT_SHOW; i++) {
            int function;
            asynStatus status;
            char  auxBitname[64];
            snprintf(auxBitname, sizeof(auxBitname), "%s_NamAuxBit%u", paramName, i);
            status = findParam(auxBitname, &function);
            if (status == asynSuccess) {
              asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                        "%s%s exist function=%d\n",
                        modNamEMC, auxBitname, function);
            } else {
              status = createParam(auxBitname, asynParamOctet, &function);
              asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                        "%s%s(%u) numPilsAsynDevInfo=%d created function=%d auxBitname=%s status=%s (%d)\n",
                        modNamEMC, auxBitname, axisNo, numPilsAsynDevInfo, function,
                        auxBitname,
                        ethercatmcstrStatus(status), (int)status);
            }
            if (status == asynSuccess && i == 0)
              functionNamAux0 = function;
          }
        }
      }
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
                             functionNamAux0,
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

void
ethercatmcController::changedAuxBits_to_ASCII(int         axisNo,
                                              int         functionNamAux0,
                                              epicsUInt32 statusReasonAux,
                                              epicsUInt32 oldStatusReasonAux)
{
  /* Show even bit 24 and 25, which are reson bits, here */
  epicsUInt32 changed = statusReasonAux ^ oldStatusReasonAux;
  epicsUInt32 auxBitIdx;
  memset(&ctrlLocal.changedAuxBits, 0, sizeof(ctrlLocal.changedAuxBits));
  for (auxBitIdx = 0; auxBitIdx < MAX_REASON_AUX_BIT_SHOW; auxBitIdx++) {
    if ((changed >> auxBitIdx) & 0x01) {
      asynStatus status = asynError;
      size_t length = sizeof(ctrlLocal.changedAuxBits[auxBitIdx]) - 2;
      if (functionNamAux0) {
        int function = (int)(functionNamAux0 + auxBitIdx);
        /* Leave the first character for '+' or '-',
           leave one byte for '\0' */
        status = getStringParam(axisNo,
                                function,
                                (int)length,
                                &ctrlLocal.changedAuxBits[auxBitIdx][1]);
      }
      if (status != asynSuccess) {
        snprintf(&ctrlLocal.changedAuxBits[auxBitIdx][1],
                 length, "Bit%d", auxBitIdx);
      }
      if ((statusReasonAux >> auxBitIdx) & 0x01) {
        ctrlLocal.changedAuxBits[auxBitIdx][0] = '+';
      } else {
        ctrlLocal.changedAuxBits[auxBitIdx][0] = '-';
      }
    }
  }
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
        const char *paramName = "";
        getParamName(axisNo, function, &paramName);

        void *pDataInPlc = &ctrlLocal.pIndexerProcessImage[inputOffset];
        asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                  "%sindexerPoll(%d) numPilsAsynDevInfo=%u inputOffset=%u\n",
                  modNamEMC, axisNo,
                  numPilsAsynDevInfo, inputOffset);
        if (statusOffset) {
          /* Add a printout for the changed AUX bits.
             currently the poller for the axis has simiar code */
          const static epicsUInt32 maskStatusReasonAux = 0x03FFFFFF;
          void *pStatusInPlc = &ctrlLocal.pIndexerProcessImage[statusOffset];
          epicsUInt32 statusReasonAux;
          unsigned statusLenInPLC = sizeof(statusReasonAux);
          int function = pPilsAsynDevInfo->function;
          if (!function) {
            function = ethercatmcStatusBits_;
          }
          statusReasonAux = netToUint(pStatusInPlc, statusLenInPLC);
          int functionNamAux0 = pPilsAsynDevInfo->functionNamAux0;
          if (functionNamAux0) {
            epicsUInt32 oldStatusReasonAux;
            getUIntDigitalParam(axisNo, function,
                                &oldStatusReasonAux, maskStatusReasonAux);
            if (statusReasonAux != oldStatusReasonAux) {
              changedAuxBits_to_ASCII(axisNo, functionNamAux0,
                                      statusReasonAux, oldStatusReasonAux);
              asynPrint(pasynUserController_, traceMask | ASYN_TRACE_INFO,
                        "%spoll(%d) %sOld=0x%04X new=0x%04X (%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s)\n",
                        modNamEMC, axisNo, paramName, oldStatusReasonAux, statusReasonAux,
                        ctrlLocal.changedAuxBits[0],  ctrlLocal.changedAuxBits[1],
                        ctrlLocal.changedAuxBits[2],  ctrlLocal.changedAuxBits[3],
                        ctrlLocal.changedAuxBits[4],  ctrlLocal.changedAuxBits[5],
                        ctrlLocal.changedAuxBits[6],  ctrlLocal.changedAuxBits[7],
                        ctrlLocal.changedAuxBits[8],  ctrlLocal.changedAuxBits[9],
                        ctrlLocal.changedAuxBits[10], ctrlLocal.changedAuxBits[11],
                        ctrlLocal.changedAuxBits[12], ctrlLocal.changedAuxBits[13],
                        ctrlLocal.changedAuxBits[14], ctrlLocal.changedAuxBits[15],
                        ctrlLocal.changedAuxBits[16], ctrlLocal.changedAuxBits[17],
                        ctrlLocal.changedAuxBits[18], ctrlLocal.changedAuxBits[19],
                        ctrlLocal.changedAuxBits[20], ctrlLocal.changedAuxBits[21],
                        ctrlLocal.changedAuxBits[22], ctrlLocal.changedAuxBits[23],
                        ctrlLocal.changedAuxBits[24], ctrlLocal.changedAuxBits[25]);
            }
          }
          setUIntDigitalParam(axisNo, function,
                              (epicsUInt32)statusReasonAux,
                              maskStatusReasonAux, maskStatusReasonAux);
        }
        if (!inputOffset) continue;


        /* Each axis has it's own parameters.
           axisNo == 0 is no special axis, parameters
           for "the controller", additional IO, PTP info */
        switch (pPilsAsynDevInfo->myEPICSParamType) {
        case asynParamInt32:
          {
            int tracelevel = ASYN_TRACE_FLOW;
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            epicsInt32 newValue, oldValue;
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
          epicsTimeStamp timeMCU;
          epicsTimeStamp timeIOC;
          unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
          nSec = netToUint64(pDataInPlc, lenInPLC);
          UTCtimeToEpicsTimeStamp(nSec, &timeMCU);
          asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
                    "%sindexerPoll SystemUTCtime nSec=%" PRIu64 " sec:nSec=%09u.%09u\n",
                    modNamEMC, nSec,
                    timeMCU.secPastEpoch, timeMCU.nsec);
          setTimeStamp(&timeMCU);
          callBacksNeeded = 1;
          int function = ethercatmcPTPdiffTimeIOC_MCU_;
          int axisNo = 0;
          int rtn = epicsTimeGetCurrent(&timeIOC);
          if (!rtn) {
            double diffTimeIOC_MCU = timeIOC.secPastEpoch - timeMCU.secPastEpoch;
            diffTimeIOC_MCU = diffTimeIOC_MCU * 1000; // msec
            diffTimeIOC_MCU += ((double)timeIOC.nsec - (double)timeMCU.nsec) / 1000000.0; // nsec -> msec
            (void)setDoubleParam(axisNo, function, diffTimeIOC_MCU);
          } else {
            setAlarmStatusSeverityWrapper(axisNo, function, asynDisconnected);
          }
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
  if (!ctrlLocal.oldStatus || ctrlLocal.numPilsAsynDevInfo) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s\n",
              modNamEMC, "indexerDisconnected");
  }
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

