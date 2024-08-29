/*
  FILENAME... ethercatmcIndexer.cpp
*/

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define __STDC_FORMAT_MACROS
#include <epicsThread.h>
#include <inttypes.h>

#include "ethercatmcController.h"
#include "ethercatmcIndexerAxis.h"

/* Alarm definition from EPICS Base */
#include <alarm.h>

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO 0x0040
#endif

/* Sleep time and max counter for communication*/
#define MAX_COUNTER 14

/*
 * Calculation of sleep time when the PLC answers/answers with "interface busy"
 * or retry later
 */
extern "C" {
double calcSleep(int counter) {
  static double sleepTime = 0.005; /* 5 msec */
  return sleepTime * (counter << 1);
}
};

extern "C" {
const char *paramIfCmdToString(unsigned cmdSubParamIndex) {
  switch (cmdSubParamIndex & PARAM_IF_CMD_MASK) {
    case PARAM_IF_CMD_INVALID:
      return "PARAM_IF_CMD_INVALID";
    case PARAM_IF_CMD_DOREAD:
      return "PARAM_IF_CMD_DOREAD";
    case PARAM_IF_CMD_DOWRITE:
      return "PARAM_IF_CMD_DOWRITE";
    case PARAM_IF_CMD_BUSY:
      return "PARAM_IF_CMD_BUSY";
    case PARAM_IF_CMD_DONE:
      return "PARAM_IF_CMD_DONE";
    case PARAM_IF_CMD_ERR_NO_IDX:
      return "PARAM_IF_CMD_ERR_NO_IDX";
    case PARAM_IF_CMD_ERR_READONLY:
      return "PARAM_IF_CMD_ERR_READONLY";
    case PARAM_IF_CMD_ERR_RETRY_LATER:
      return "PARAM_IF_CMD_ERR_RETRY_LATER";
    default:
      return "cmdSubParamIndexXXXX";
  }
}
}

const char *ethercatmcController::plcParamIndexTxtFromParamIndex(
    unsigned cmdSubParamIndex) {
  unsigned paramIndex = cmdSubParamIndex & PARAM_IF_IDX_MASK;
  switch (paramIndex) {
    case 0:
      return "PARAM_ZERO";
    case PARAM_IDX_OPMODE_AUTO_UINT:
      return "OPMODE_AUTO";
    case PARAM_IDX_MICROSTEPS_UINT:
      return "MICROSTEPS";
    case PARAM_IDX_ABS_MIN_FLOAT:
      return "ABS_MIN";
    case PARAM_IDX_ABS_MAX_FLOAT:
      return "ABS_MAX";
    case PARAM_IDX_USR_MIN_FLOAT:
      return "USR_MIN";
    case PARAM_IDX_USR_MAX_FLOAT:
      return "USR_MAX";
    case PARAM_IDX_WRN_MIN_FLOAT:
      return "WRN_MIN";
    case PARAM_IDX_WRN_MAX_FLOAT:
      return "WRN_MAX";
    case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
      return "FOLLOWING_ERR_WIN";
    case PARAM_IDX_HYTERESIS_FLOAT:
      return "HYTERESIS";
    case PARAM_IDX_REFSPEED_FLOAT:
      return "REFSPEED";
    case PARAM_IDX_VBAS_FLOAT:
      return "VBAS";
    case PARAM_IDX_SPEED_FLOAT:
      return "SPEED";
    case PARAM_IDX_ACCEL_FLOAT:
      return "ACCEL";
    case PARAM_IDX_IDLE_CURRENT_FLOAT:
      return "IDLE_CURRENT";
    case PARAM_IDX_MOVE_CURRENT_FLOAT:
      return "MOVE_CURRENT";
    case PARAM_IDX_MICROSTEPS_FLOAT:
      return "MICROSTEPS";
    case PARAM_IDX_STEPS_PER_UNIT_FLOAT:
      return "STEPS_PER_UNIT";
    case PARAM_IDX_HOME_POSITION_FLOAT:
      return "HOME_POSITION";
    case PARAM_IDX_FUN_REFERENCE:
      return "REFERENCE";
    case PARAM_IDX_FUN_SET_POSITION:
      return "SET_POSITION";
    case PARAM_IDX_FUN_MOVE_VELOCITY:
      return "MOVE_VELOCITY";
    case PARAM_IDX_USR_MIN_EN_FLOAT:
      return "USR_MIN_EN";
    case PARAM_IDX_USR_MAX_EN_FLOAT:
      return "USR_MAX_EN";
    case PARAM_IDX_HOMPROC_FLOAT:
      return "HOMPROC";
    case PARAM_IDX_UNITS_PER_REV_FLOAT:
      return "UNITS_PER_REV";
    case PARAM_IDX_STEPS_PER_REV_FLOAT:
      return "STEPS_PER_REV";
    case PARAM_IDX_MAX_VELO_FLOAT:
      return "MAX_VELO";
    default:
      if ((paramIndex < (sizeof(ctrlLocal.functionFromParamIndex) /
                         sizeof(ctrlLocal.functionFromParamIndex[0]))) &&
          ctrlLocal.functionFromParamIndex[paramIndex]) {
        if (paramIndex >= PARAM_IF_IDX_FIRST_CUSTOM_PARA &&
            paramIndex <= PARAM_IF_IDX_LAST_CUSTOM_PARA) {
          unsigned paramIndexCustom =
              paramIndex - PARAM_IF_IDX_FIRST_CUSTOM_PARA;
          return (const char *)&ctrlLocal.customParaName[paramIndexCustom];
        }
      }
      return "PX";
  }
};

extern "C" {
int paramIndexIsIntegerV2(unsigned paramIndex) {
  if (paramIndex < 30) {
    /* parameters below 30 are unsigned integers in the PLC */
    return 1;
  } else if (paramIndex >= PARAM_IF_IDX_FIRST_CUSTOM_PARA &&
             paramIndex <= 194) {
    /*
       Parameters 192 .. 194 had been defined as integers
       some time ago. Keep this as legacy.
       See customParaName in ethercatmcIndexerV2.cpp
    */
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
    case PARAM_IDX_UNITS_PER_REV_FLOAT:
    case PARAM_IDX_STEPS_PER_REV_FLOAT:
    case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
      return 1;
    default:
      if ((paramIndex >= PARAM_IF_IDX_FIRST_CUSTOM_PARA &&
           paramIndex <= PARAM_IF_IDX_LAST_CUSTOM_PARA)) {
        return 1;
      }
      return 0;
  }
}
};

/* Wrapper methods: if something fails, change state */
asynStatus ethercatmcController::getPlcMemoryOnErrorStateChangeFL(
    unsigned indexOffset, void *data, size_t lenInPlc, const char *fileName,
    int lineNo) {
  asynStatus status;
  status = getPlcMemoryViaADSFL(indexOffset, data, lenInPlc, fileName, lineNo);
  if (status) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%sgetPlcMemoryOnErrorStateChangeFL %s:%d status=%s (%d) "
              "oldStatus=%d\n",
              modNamEMC, fileName, lineNo, ethercatmcstrStatus(status),
              (int)status, (int)ctrlLocal.oldStatus);
    handleStatusChange(status);
  }
  return status;
}

asynStatus ethercatmcController::setPlcMemoryOnErrorStateChangeFL(
    unsigned indexOffset, const void *data, size_t lenInPlc,
    const char *fileName, int lineNo) {
  asynStatus status;
  status = setPlcMemoryViaADSFL(indexOffset, data, lenInPlc, fileName, lineNo);
  if (status) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%ssetPlcMemoryOnErrorStateChangeFL %s:%d status=%s (%d) "
              "oldStatus=%d\n",
              modNamEMC, fileName, lineNo, ethercatmcstrStatus(status),
              (int)status, (int)ctrlLocal.oldStatus);
    handleStatusChange(status);
  }
  return status;
}
/* end of wrappers */

/* Re-define calles without FILE and LINE */
#define getPlcMemoryOnErrorStateChange(a, b, c) \
  getPlcMemoryOnErrorStateChangeFL(a, b, c, __FILE__, __LINE__)
#define setPlcMemoryOnErrorStateChange(a, b, c) \
  setPlcMemoryOnErrorStateChangeFL(a, b, c, __FILE__, __LINE__)

/* No "direct" calls into ADS below this point */
#undef getPlcMemoryViaADS
#undef setPlcMemoryViaADS
#define getPlcMemoryViaADS #error
#define setPlcMemoryViaADS #error
#define getPlcMemoryViaADSFL #error
#define setPlcMemoryViaADSFL #error

asynStatus ethercatmcController::getPlcMemoryUintFL(unsigned indexOffset,
                                                    unsigned *value,
                                                    size_t lenInPlc,
                                                    const char *fileName,
                                                    int lineNo) {
  asynStatus status;

  memset(value, 0, lenInPlc);
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    status = getPlcMemoryOnErrorStateChangeFL(indexOffset, &raw, lenInPlc,
                                              fileName, lineNo);
    *value = netToUint(&raw, lenInPlc);
    return status;
  }
  return asynError;
}

asynStatus ethercatmcController::setPlcMemoryInteger(unsigned indexOffset,
                                                     int value,
                                                     size_t lenInPlc) {
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
                                                    size_t lenInPlc) {
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
                                                    size_t lenInPlc) {
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    doubleToNet(value, &raw, lenInPlc);
    return setPlcMemoryOnErrorStateChange(indexOffset, &raw, lenInPlc);
  } else {
    return asynError;
  }
}

asynStatus ethercatmcController::setSAFIntegerOnAxisViaADSFL(
    unsigned indexGroup, unsigned indexOffset, int value, size_t lenInPlc,
    const char *fileName, int lineNo) {
  const static unsigned targetAdsport = 501;
  asynStatus status;
  if (lenInPlc <= 8) {
    uint8_t raw[8];
    uintToNet(value, &raw, lenInPlc);

    status = setMemIdxGrpIdxOffFL(indexGroup, indexOffset, targetAdsport, &raw,
                                  lenInPlc, fileName, lineNo);
    asynPrint(
        pasynUserController_, ASYN_TRACE_INFO,
        "%s%s:%d setSAFIntegerOnAxisViaADSFL indexGroup=0x%X indexOffset=0x%X"
        " value=%d lenInPlc=%u status=%s (%d)\n",
        modNamEMC, fileName, lineNo, indexGroup, indexOffset, value,
        (unsigned)lenInPlc, ethercatmcstrStatus(status), (int)status);
    return status;
  } else {
    return asynError;
  }
}

asynStatus ethercatmcController::setSAFDoubleOnAxisViaADSFL(
    unsigned indexGroup, unsigned indexOffset, double value, size_t lenInPlc,
    const char *fileName, int lineNo) {
  const static unsigned targetAdsport = 501;
  asynStatus status;
  if (lenInPlc == 8 || lenInPlc == 4) {
    uint8_t raw[8];
    doubleToNet(value, &raw, lenInPlc);

    status = setMemIdxGrpIdxOffFL(indexGroup, indexOffset, targetAdsport, &raw,
                                  lenInPlc, fileName, lineNo);
    asynPrint(
        pasynUserController_, ASYN_TRACE_INFO,
        "%s%s:%d setSAFIntegerOnAxisViaADSFL indexGroup=0x%X indexOffset=0x%X"
        " value=%f lenInPlc=%u status=%s (%d)\n",
        modNamEMC, fileName, lineNo, indexGroup, indexOffset, value,
        (unsigned)lenInPlc, ethercatmcstrStatus(status), (int)status);
    return status;
  } else {
    return asynError;
  }
}

asynStatus ethercatmcController::indexerWaitSpecialDeviceIdle(
    unsigned indexOffset) {
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned ctrlLen = 0;
  unsigned counter = 0;
  unsigned plcNotHostHasWritten;

  while (counter < MAX_COUNTER) {
    status = getPlcMemoryUint(indexOffset, &ctrlLen, 2);
    plcNotHostHasWritten = (ctrlLen & 0x8000) ? 1 : 0;

    asynPrint(pasynUserController_,
              status ? traceMask | ASYN_TRACE_INFO : traceMask,
              "%sindexerWaitSpecialDeviceIdle ctrlLen=0x%04x status=%s (%d)\n",
              modNamEMC, ctrlLen, ethercatmcstrStatus(status), (int)status);
    if (status) return status;
    if (plcNotHostHasWritten) return asynSuccess;
    counter++;
    epicsThreadSleep(calcSleep(counter));
  }
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%sindexOffset=%u ctrlLen=0x%04X counter=%d\n", modNamEMC,
            indexOffset, ctrlLen, counter);
  return asynDisabled;
}

asynStatus ethercatmcController::indexerParamReadFL(
    ethercatmcIndexerAxis *pAxis, unsigned paramIfOffset, unsigned paramIndex,
    double *value, const char *fileName, int lineNo) {
  paramIf_type paramIf_from_MCU;
  unsigned traceMask = ASYN_TRACE_FLOW;
  asynStatus status;
  unsigned cmd = PARAM_IF_CMD_DOREAD + paramIndex;
  unsigned counter = 0;
  unsigned lenInPlcPara = 0;
  int axisNo = pAxis->axisNo_;
  if (pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex];
  } else if (pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex];
  }
  if (!paramIfOffset || paramIndex > 0xFF || !lenInPlcPara ||
      lenInPlcPara > sizeof(paramIf_from_MCU.paramValueRaw)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%sindexerParamRead(%d) paramIndex=%u lenInPlcPara=%u "
              "paramIfOffset=%u\n",
              modNamEMC, axisNo, paramIndex, lenInPlcPara, paramIfOffset);
    return asynDisabled;
  }
  size_t lenInPLCparamIf = sizeof(paramIf_from_MCU.paramCtrl) + lenInPlcPara;
  while (counter < MAX_COUNTER) {
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    status = getPlcMemoryOnErrorStateChange(paramIfOffset, &paramIf_from_MCU,
                                            lenInPLCparamIf);

    if (status) {
      asynPrint(
          pasynUserController_, traceMask | ASYN_TRACE_INFO,
          "%s:%d %s(%d) paramIfOffset=%u lenInPLCparamIf=%u status=%s (%d)\n",
          fileName, lineNo, "indexerParamRead", axisNo, paramIfOffset,
          (unsigned)lenInPLCparamIf, ethercatmcstrStatus(status), (int)status);
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
                plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                paramIndex, counter, paramIfCmdToString(cmdSubParamIndexRB),
                cmdSubParamIndexRB);
    }
    switch (cmdSubParamIndexRB & PARAM_IF_CMD_MASK) {
      case PARAM_IF_CMD_DONE:
        if (paramIndexRB == paramIndex) {
          /* This is good, return */
          double fValue = -1;  // NaN;
          if (pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex]) {
            fValue = (double)netToUint(&paramIf_from_MCU.paramValueRaw,
                                       lenInPlcPara);
          } else if (pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex]) {
            fValue = netToDouble(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
          }
          asynPrint(
              pasynUserController_, traceMask /* | ASYN_TRACE_INFO */,
              "%s:%d %s(%d) paramIfOffset=%u paramIdxFunction=%s (%u 0x%02X) "
              "lenInPlcParaFloat=%u lenInPlcParaInteger=%u lenInPlcPara=%u "
              "value=%f\n",
              fileName, lineNo, "indexerParamRead", pAxis->axisNo_,
              paramIfOffset, plcParamIndexTxtFromParamIndex(paramIndex),
              paramIndex, paramIndex,
              pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex],
              pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex],
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
          status = setPlcMemoryInteger(
              paramIfOffset, cmd, (unsigned)sizeof(paramIf_from_MCU.paramCtrl));
          if (status) return status;
        } else {
          int axisNo = pAxis->axisNo_;
          status = asynDisabled;
          asynPrint(
              pasynUserController_, traceMask | ASYN_TRACE_INFO,
              "%s:%d %s(%d) paramIfOffset=%u paramIdxFunction=%s (%u 0x%02X) "
              "cmdSubParamIndexRB=%s (0x%04X) status=%s (%d)\n",
              fileName, lineNo, "indexerParamRead", axisNo, paramIfOffset,
              plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
              paramIndex, paramIfCmdToString(cmdSubParamIndexRB),
              cmdSubParamIndexRB, ethercatmcstrStatus(status), (int)status);
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
                                                   double *pValueRB) {
  int axisNo = pAxis->axisNo_;
  paramIf_type paramIf_to_MCU;
  paramIf_type paramIf_from_MCU;
  struct {
    uint8_t actPos[8];
    uint8_t targtPos[8];
    uint8_t statReasAux[4];
    uint8_t errorID[2];
    uint8_t paramCtrl[2];
    uint8_t paramValue[8];
  } readback_5010;

  unsigned traceMask = ASYN_TRACE_INFO;
  asynStatus status = asynSuccess;
  unsigned cmd = PARAM_IF_CMD_DOWRITE + paramIndex;
  unsigned counter = 0;
  int has_written = 0;
  unsigned lenInPlcPara = 0;
  unsigned paramIfOffset = pAxis->drvlocal.clean.paramIfOffset;

  if (pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex];
  } else if (pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex]) {
    lenInPlcPara = pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex];
  }
  if (!pAxis || !paramIfOffset || (paramIndex > 0xFF) ||
      lenInPlcPara > sizeof(paramIf_to_MCU.paramValueRaw)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%s pAxis=%p paramIndex=%u lenInPlcPara=%u paramIfOffset=%u "
              "pAxis=%p paramIfOffset=%u paramIndex=%u\n",
              modNamEMC, pAxis, paramIndex, lenInPlcPara, paramIfOffset, pAxis,
              paramIfOffset, paramIndex);
    return asynError;
  } else if ((pAxis->drvlocal.clean.PILSparamPerm[paramIndex] ==
              PILSparamPermRead) ||
             (pAxis->drvlocal.clean.PILSparamPerm[paramIndex] ==
              PILSparamPermNone)) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%s pAxis=%p paramIndex=%u lenInPlcPara=%u paramIfOffset=%u "
              "perm=%d\n",
              modNamEMC, pAxis, paramIndex, lenInPlcPara, paramIfOffset,
              (int)pAxis->drvlocal.clean.PILSparamPerm[paramIndex]);
    return asynError;
  }
  size_t lenInPLCparamIf = sizeof(paramIf_to_MCU.paramCtrl) + lenInPlcPara;

  memset(&paramIf_to_MCU, 0, sizeof(paramIf_to_MCU));
  memset(&paramIf_from_MCU, 0, sizeof(paramIf_from_MCU));
  memset(&readback_5010, 0, sizeof(readback_5010));

  if (pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex]) {
    uintToNet((int)value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  } else if (pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex]) {
    doubleToNet(value, &paramIf_to_MCU.paramValueRaw, lenInPlcPara);
  }
  UINTTONET(cmd, paramIf_to_MCU.paramCtrl);

  while (counter < MAX_COUNTER) {
    /* get the paraminterface "as is". It may be in DONE state as an answer
       to a write from a previous round */
    if (pAxis && pAxis->drvlocal.clean.iTypCode == 0x5010 &&
        !pAxis->drvlocal.dirty.initialPollNeeded) {
      status = getPlcMemoryOnErrorStateChange(
          pAxis->drvlocal.clean.iOffset, &readback_5010, sizeof(readback_5010));
      if (status) return status;
      memcpy(&paramIf_from_MCU, &readback_5010.paramCtrl,
             sizeof(paramIf_from_MCU));
    } else {
      status = getPlcMemoryOnErrorStateChange(paramIfOffset, &paramIf_from_MCU,
                                              lenInPLCparamIf);
    }
    if (status) return status;
    double valueRB = -1.0;
    if (pAxis->drvlocal.clean.lenInPlcParaInteger[paramIndex]) {
      valueRB = netToUint(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    } else if (pAxis->drvlocal.clean.lenInPlcParaFloat[paramIndex]) {
      valueRB = netToDouble(&paramIf_from_MCU.paramValueRaw, lenInPlcPara);
    }
    unsigned cmdSubParamIndexRB = NETTOUINT(paramIf_from_MCU.paramCtrl);
    unsigned paramIndexRB = cmdSubParamIndexRB & PARAM_IF_IDX_MASK;

    if (counter >= 1) {
      asynPrint(
          pasynUserController_, traceMask,
          "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g "
          "counter=%u RB=%s (0x%04X)\n",
          modNamEMC, axisNo, plcParamIndexTxtFromParamIndex(paramIndex),
          paramIndex, paramIndex, value, counter,
          paramIfCmdToString(cmdSubParamIndexRB),
          // Print the even the sub-index as HEX, in case the PLC "hides" some
          // info here
          cmdSubParamIndexRB & (PARAM_IF_SUBIDX_MASK | PARAM_IF_IDX_MASK));
    }
    unsigned paramIfCmd = cmdSubParamIndexRB & PARAM_IF_CMD_MASK;
    switch (paramIfCmd) {
      case PARAM_IF_CMD_DONE: {
        if (paramIndexRB == paramIndex) {
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g "
                    "valueRB=%02g has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                    paramIndex, value, valueRB, has_written);
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
      case PARAM_IF_CMD_ERR_RETRY_LATER: {
        /* param interface is not busy */
        if (paramIndexRB != paramIndex || !has_written) {
          /* Send the write request, unless we already done it */
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g "
                    "lenInPlcPara=%u has_written=%d\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                    paramIndex, value, lenInPlcPara, has_written);
          status = setPlcMemoryOnErrorStateChange(
              paramIfOffset, &paramIf_to_MCU, (unsigned)sizeof(paramIf_to_MCU));
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
                pAxis->drvlocal.clean.PILSparamPerm[paramIndex] =
                    PILSparamPermRead;
              }
              status = asynParamWrongType;
            }
          }
        }
        if (status != asynSuccess) {
          asynPrint(pasynUserController_,
                    ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                    "%s pAxis=%p paramIndex=%u lenInPlcPara=%u "
                    "paramIfOffset=%u status=%s (%d)\n",
                    modNamEMC, pAxis, paramIndex, lenInPlcPara, paramIfOffset,
                    ethercatmcstrStatus(status), (int)status);
          goto indexerParamWritePrintAuxReturn;
        }
      } break;
      case PARAM_IF_CMD_BUSY: {
        /* A "function" goes into busy - and stays there */
        /* No parameter settings during jogging/homing */
        if (paramIndexIsMovingFunction(paramIndex)) {
          asynPrint(pasynUserController_, traceMask,
                    "%sindexerParamWrite(%d) %s(%u 0x%02X) value=%02g "
                    "movingFun RB=%s (0x%04X)\n",
                    modNamEMC, axisNo,
                    plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                    paramIndex, value, paramIfCmdToString(cmdSubParamIndexRB),
                    cmdSubParamIndexRB);
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
  status = asynDisabled;
  asynPrint(pasynUserController_, traceMask | ASYN_TRACE_INFO, "%scounter=%u\n",
            modNamEMC, counter);

indexerParamWritePrintAuxReturn:
  if (pAxis && pAxis->drvlocal.clean.iTypCode == 0x5010 &&
      !pAxis->drvlocal.dirty.initialPollNeeded) {
    unsigned statusReasonAux = NETTOUINT(readback_5010.statReasAux);
    int errorID = (int)NETTOUINT(readback_5010.errorID);
    idxStatusCodeType idxStatusCode =
        (idxStatusCodeType)(statusReasonAux >> 28);
    // unsigned idxReasonBits = (statusReasonAux >> 24) & 0x0F;
    unsigned idxAuxBits = statusReasonAux & 0x03FFFFFF;
    if (idxAuxBits != pAxis->drvlocal.clean.old_idxAuxBitsPrinted) {
      changedAuxBits_to_ASCII(pAxis->axisNo_, defAsynPara.ethercatmcNamAux0_,
                              idxAuxBits,
                              pAxis->drvlocal.clean.old_idxAuxBitsPrinted);
      asynPrint(
          pasynUserController_, traceMask,
          "%sindexerParamWrite(%d) idxStatusCode=0x%02X auxBitsOld=0x%06X "
          "new=0x%06X (%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s) "
          "errorID=0x%04X \"%s\" \n",
          modNamEMC, pAxis->axisNo_, idxStatusCode,
          pAxis->drvlocal.clean.old_idxAuxBitsPrinted, idxAuxBits,
          ctrlLocal.changedAuxBits[0], ctrlLocal.changedAuxBits[1],
          ctrlLocal.changedAuxBits[2], ctrlLocal.changedAuxBits[3],
          ctrlLocal.changedAuxBits[4], ctrlLocal.changedAuxBits[5],
          ctrlLocal.changedAuxBits[6], ctrlLocal.changedAuxBits[7],
          ctrlLocal.changedAuxBits[8], ctrlLocal.changedAuxBits[9],
          ctrlLocal.changedAuxBits[10], ctrlLocal.changedAuxBits[11],
          ctrlLocal.changedAuxBits[12], ctrlLocal.changedAuxBits[13],
          ctrlLocal.changedAuxBits[14], ctrlLocal.changedAuxBits[15],
          ctrlLocal.changedAuxBits[16], ctrlLocal.changedAuxBits[17],
          ctrlLocal.changedAuxBits[18], ctrlLocal.changedAuxBits[19],
          ctrlLocal.changedAuxBits[20], ctrlLocal.changedAuxBits[21],
          ctrlLocal.changedAuxBits[22], ctrlLocal.changedAuxBits[23],
          ctrlLocal.changedAuxBits[24], ctrlLocal.changedAuxBits[25], errorID,
          errStringFromErrId(errorID));
      pAxis->drvlocal.clean.old_idxAuxBitsPrinted = idxAuxBits;
    }
  }
  return status;
}

asynStatus ethercatmcController::getPlcMemoryFromProcessImage(
    unsigned indexOffset, void *data, size_t lenInPlc) {
  if (ctrlLocal.pIndexerProcessImage && ctrlLocal.lastDeviceEndOffset) {
    memcpy(data, &ctrlLocal.pIndexerProcessImage[indexOffset], lenInPlc);
    return asynSuccess;
  }
  /* no process image, get values */
  return asynDisabled;
}

int ethercatmcController::paramIndexToFunction(unsigned paramIndex) {
  switch (paramIndex) {
    case PARAM_IDX_OPMODE_AUTO_UINT:
      /* CNEN for EPICS */
      return motorStatusGainSupport_;
    case PARAM_IDX_MICROSTEPS_FLOAT:
      return defAsynPara.ethercatmcCfgSREV_RB_;
    case PARAM_IDX_USR_MIN_FLOAT:
      return defAsynPara.ethercatmcCfgDLLM_RB_;
    case PARAM_IDX_ABS_MIN_FLOAT:
      return defAsynPara.ethercatmcCfgPMIN_RB_;
    case PARAM_IDX_ABS_MAX_FLOAT:
      return defAsynPara.ethercatmcCfgPMAX_RB_;
    case PARAM_IDX_USR_MAX_FLOAT:
      return defAsynPara.ethercatmcCfgDHLM_RB_;
    // case PARAM_IDX_WRN_MIN_FLOAT:
    // case PARAM_IDX_WRN_MAX_FLOAT:
    case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
      return defAsynPara.ethercatmcCfgPOSLAG_RB_;
    case PARAM_IDX_HYTERESIS_FLOAT:
      return defAsynPara.ethercatmcCfgSPDB_RB_;
      // return defAsynPara.ethercatmcCfgRDBD_RB_;
    case PARAM_IDX_REFSPEED_FLOAT:
      return defAsynPara.ethercatmcCfgHVEL_RB_;
    case PARAM_IDX_SPEED_FLOAT:
      return defAsynPara.ethercatmcVel_RB_;
    case PARAM_IDX_ACCEL_FLOAT:
      return defAsynPara.ethercatmcAcc_RB_;
      // case PARAM_IDX_IDLE_CURRENT_FLOAT:
      // case PARAM_IDX_MOVE_CURRENT_FLOAT:
      // case PARAM_IDX_MICROSTEPS_UINT:
      // case PARAM_IDX_STEPS_PER_UNIT_FLOAT:
      return defAsynPara.ethercatmcCfgUREV_RB_;
    case PARAM_IDX_HOME_POSITION_FLOAT:
      return defAsynPara.ethercatmcHomPos_RB_;
    // case PARAM_IDX_FUN_REFERENCE:
    // case PARAM_IDX_FUN_SET_POSITION:
    case PARAM_IDX_FUN_MOVE_VELOCITY:
      return defAsynPara.ethercatmcCfgJVEL_RB_;
    case PARAM_IDX_USR_MIN_EN_FLOAT:
      return defAsynPara.ethercatmcCfgDLLM_En_;
    case PARAM_IDX_USR_MAX_EN_FLOAT:
      return defAsynPara.ethercatmcCfgDHLM_En_;
    case PARAM_IDX_HOMPROC_FLOAT:
      return defAsynPara.ethercatmcHomProc_RB_;
    case PARAM_IDX_UNITS_PER_REV_FLOAT:
      return defAsynPara.ethercatmcCfgUREV_RB_;
    case PARAM_IDX_STEPS_PER_REV_FLOAT:
      return defAsynPara.ethercatmcCfgSREV_RB_;
    case PARAM_IDX_MAX_VELO_FLOAT:
      return defAsynPara.ethercatmcCfgVMAX_RB_;
    default:
      if (paramIndex < (sizeof(ctrlLocal.functionFromParamIndex) /
                        sizeof(ctrlLocal.functionFromParamIndex[0]))) {
        return ctrlLocal.functionFromParamIndex[paramIndex];
      }
      return 0;
  }
}

void ethercatmcController::parameterFloatReadBack(unsigned axisNo, int initial,
                                                  unsigned paramIndex,
                                                  double fValue) {
  const static double fullsrev = 200; /* (default) Full steps/revolution */

  switch (paramIndex) {
    case PARAM_IDX_MICROSTEPS_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgSREV_RB_,
                     fullsrev * fValue, "srev");
      break;
    case PARAM_IDX_USR_MIN_FLOAT: {
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDLLM_RB_, fValue, "dllm");
      if (initial)
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDLLM_, fValue,
                       "CfgDllm");
    }
      udateMotorLimitsRO(axisNo);
      break;
    case PARAM_IDX_ABS_MIN_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgPMIN_RB_, fValue,
                     "posmin");
      break;
    case PARAM_IDX_ABS_MAX_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgPMAX_RB_, fValue,
                     "posmax");
      break;
    case PARAM_IDX_USR_MAX_FLOAT: {
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDHLM_RB_, fValue, "dhlm");
      if (initial)
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDHLM_, fValue,
                       "CfgDhlm");
    }
      udateMotorLimitsRO(axisNo);
      break;
    case PARAM_IDX_WRN_MIN_FLOAT:
      break;
    case PARAM_IDX_WRN_MAX_FLOAT:
      break;
    case PARAM_IDX_FOLLOWING_ERR_WIN_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgPOSLAG_RB_, fValue,
                     "poslag");
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgPOSLAG_En_RB_,
                     fValue > 0.0, "poslag_en");
      break;
    case PARAM_IDX_HYTERESIS_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgSPDB_RB_, fValue, "spdb");
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgRDBD_RB_, fValue, "rdbd");
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgRDBD_En_RB_, fValue > 0.0,
                     "rdbd_en");
      break;
    case PARAM_IDX_REFSPEED_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgHVEL_RB_, fValue, "hvel");
      break;
    case PARAM_IDX_SPEED_FLOAT:
      /* velocity, the way we use it, has 2 purposes: veloRB and veloCFG.
         Note that veloCFG must always be positive. veloRB may be negative */
      if (initial)
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgVELO_RB_, fabs(fValue),
                       "veloCFG");
      updateCfgValue(axisNo, defAsynPara.ethercatmcVel_RB_, fValue, "veloRB");
#ifdef motorDefVelocityROString
      pAxis->setDoubleParam(motorDefVelocityRO_, fValue);
#endif
      break;
    case PARAM_IDX_ACCEL_FLOAT:
      if (initial)
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgACCS_RB_, fValue,
                       "accsRB");
      updateCfgValue(axisNo, defAsynPara.ethercatmcAcc_RB_, fValue, "accsRB");
      break;
    case PARAM_IDX_IDLE_CURRENT_FLOAT:
      break;
    case PARAM_IDX_MOVE_CURRENT_FLOAT:
      break;
    case PARAM_IDX_MICROSTEPS_UINT:
      break;
    case PARAM_IDX_STEPS_PER_UNIT_FLOAT: {
      double urev = fabs(fullsrev / fValue);
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgUREV_RB_, urev, "urev");
    } break;
    case PARAM_IDX_HOME_POSITION_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcHomPos_RB_, fValue,
                     "homPosRB");
      break;
    case PARAM_IDX_FUN_MOVE_VELOCITY:
      if (initial)
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgJVEL_RB_, fabs(fValue),
                       "jvel");
      break;
    case PARAM_IDX_USR_MIN_EN_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDLLM_En_, (int)fValue,
                     "dllm_en");
      udateMotorLimitsRO(axisNo);
      break;
    case PARAM_IDX_USR_MAX_EN_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcCfgDHLM_En_, (int)fValue,
                     "dhlm_en");
      udateMotorLimitsRO(axisNo);
      break;
    case PARAM_IDX_HOMPROC_FLOAT:
      updateCfgValue(axisNo, defAsynPara.ethercatmcHomProc_RB_, (int)fValue,
                     "homprocRB");
      break;
    case PARAM_IDX_UNITS_PER_REV_FLOAT: {
      if (fValue > 0.0) {
        double urev = fabs(fValue);
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgUREV_RB_, urev, "urev");
      }
    } break;
    case PARAM_IDX_STEPS_PER_REV_FLOAT:
      if (fValue > 0.0) {
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgSREV_RB_, fValue,
                       "srev");
      }
      break;
    case PARAM_IDX_MAX_VELO_FLOAT:
      if (fValue > 0.0) {
        updateCfgValue(axisNo, defAsynPara.ethercatmcCfgVMAX_RB_, fValue,
                       "vmax");
      }
      break;
    default:
      int function = paramIndexToFunction(paramIndex);
      if (function) {
        updateCfgValue(axisNo, function, fValue,
                       plcParamIndexTxtFromParamIndex(paramIndex));
      }
  }
}

asynStatus ethercatmcController::indexerReadAxisParameters(
    ethercatmcIndexerAxis *pAxis, unsigned devNum) {
  unsigned axisNo = pAxis->axisNo_;
  asynStatus status = asynError;
  /* Find out which parameters that exist for this device
     The result is stored in pAxis->drvlocal.clean.PILSparamPerm[] */

  unsigned paramIfOffset = pAxis->drvlocal.clean.paramIfOffset;
  if (ctrlLocal.supported.bPILSv2) {
    status = indexerReadAxisParametersV2(pAxis, devNum);
  } else if (ctrlLocal.supported.bPILSv3) {
    status = asynSuccess; /* see indexerV3readParameterDescriptors() */
  }
  if (status) return status;
  /* loop through all parameters.
     PILS v2 and v3 use the same param interface logic */
  unsigned paramIndex;

  for (paramIndex = 0;
       paramIndex < (sizeof(pAxis->drvlocal.clean.PILSparamPerm) /
                     sizeof(pAxis->drvlocal.clean.PILSparamPerm[0]));
       paramIndex++) {
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%sindexerReadAxisParameters(%d) paramIdx=%s (%u) perm=%d\n",
              modNamEMC, axisNo, plcParamIndexTxtFromParamIndex(paramIndex),
              paramIndex, (int)pAxis->drvlocal.clean.PILSparamPerm[paramIndex]);
    if (pAxis->drvlocal.clean.PILSparamPerm[paramIndex] != PILSparamPermNone) {
      // parameter is read or write
      double fValue = 0.0;
      int initial = 1;
      switch (paramIndex) {
        case PARAM_IDX_OPMODE_AUTO_UINT:
          /* CNEN for EPICS */
          pAxis->setIntegerParam(motorStatusGainSupport_, 1);
          break;
        case PARAM_IDX_FUN_REFERENCE:
#ifdef motorNotHomedProblemString
          pAxis->setIntegerParam(motorNotHomedProblem_,
                                 MOTORNOTHOMEDPROBLEM_ERROR);
#endif
          updateCfgValue(axisNo, defAsynPara.ethercatmcHomeVis_, 1, "homeVis");
          break;
        case PARAM_IDX_FUN_SET_POSITION:
#ifdef motorNotHomedProblemString
          pAxis->setIntegerParam(motorNotHomedProblem_,
                                 MOTORNOTHOMEDPROBLEM_ERROR);
#endif
          updateCfgValue(axisNo, defAsynPara.ethercatmcFoffVis_, 1, "foffVis");
          break;
      }
      if (paramIndexIsParameterToPoll(paramIndex) ||
          (paramIndex == PARAM_IDX_FUN_MOVE_VELOCITY)) {
        /* Some parameters are functions: Don't read them.
           tell driver that the function exist
           But read 142, which becomes JVEL */
        if (pAxis->drvlocal.clean.enumparam_read_id[paramIndex]) {
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%s (%u) has enums\n", modNamEMC,
                    axisNo, plcParamIndexTxtFromParamIndex(paramIndex),
                    paramIndex);
        } else if (paramIndexIsReadLaterInBackground(paramIndex)) {
          asynPrint(
              pasynUserController_, ASYN_TRACE_INFO,
              "%sparameters(%d) paramIdx=%s (%u) only polled in background\n",
              modNamEMC, axisNo, plcParamIndexTxtFromParamIndex(paramIndex),
              paramIndex);
        } else {
          status = indexerParamRead(pAxis, paramIfOffset, paramIndex, &fValue);
          if (status) {
            asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                      "%sindexerReadAxisParameters(%d) paramIdx=%s (%u)"
                      " status=%s (%d)\n",
                      modNamEMC, axisNo,
                      plcParamIndexTxtFromParamIndex(paramIndex), paramIndex,
                      ethercatmcstrStatus(status), (int)status);
            return status;
          }
          parameterFloatReadBack(axisNo, initial, paramIndex, fValue);
          asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                    "%sparameters(%d) paramIdx=%s (%u) value=%f\n", modNamEMC,
                    axisNo, plcParamIndexTxtFromParamIndex(paramIndex),
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

asynStatus ethercatmcController::indexerInitialPoll(void) {
  asynStatus status;
  {
    ethercatmcIndexerAxis *pAxis;
    int axisNo = 0;
    pAxis = static_cast<ethercatmcIndexerAxis *>(
        asynMotorController::getAxis(axisNo));
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
  status =
      getPlcMemoryUint(ctrlLocal.indexerOffset, &ctrlLocal.indexerOffset, 2);
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
                "%sadsport=%u version='%s' 0x%X indexerOffset=%u\n", modNamEMC,
                ctrlLocal.adsport, version, iTmpVer, ctrlLocal.indexerOffset);
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

int ethercatmcController::addPilsAsynDevLst(
    int axisNo, const char *paramName, const char *paramDescField,
    int functionNamAux0, int functionStatusBits, unsigned lenInPLC,
    unsigned inputOffset, unsigned outputOffset, unsigned statusOffset,
    asynParamType myEPICSParamType, unsigned iTypCode) {
  const static char *const functionName = "addPilsAsynDevLst";
  unsigned numPilsAsynDevInfo = ctrlLocal.numPilsAsynDevInfo;
  static size_t maxNumPilsAsynDevInfo = (sizeof(ctrlLocal.pilsAsynDevInfo) /
                                         sizeof(ctrlLocal.pilsAsynDevInfo[0])) -
                                        1;

  asynStatus status;
  int function = 0;
  int functionDescField = 0;
  pilsAsynDevInfo_type *pPilsAsynDevInfo =
      &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

  if (numPilsAsynDevInfo >= maxNumPilsAsynDevInfo) {
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR, "%s%s(%u) out of range\n",
              modNamEMC, functionName, numPilsAsynDevInfo);
    return -1;
  }

  asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
            "%s%s axisNo=%i \"%s\" functionNamAux0=%d functionStatusBits=%d "
            "lenInPLC=%u inputOffset=%u outputOffset=%u statusOffset=%u"
            " EPICSParamType=%s(%i) iTypeCode=0x%04X\n",
            modNamEMC, functionName, axisNo, paramName, functionNamAux0,
            functionStatusBits, lenInPLC, inputOffset, outputOffset,
            statusOffset, stringFromAsynParamType(myEPICSParamType),
            (int)myEPICSParamType, iTypCode);
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s(%u) \"%s\" EPICSParamType=%s(%i)\n", modNamEMC, functionName,
            axisNo, paramName, stringFromAsynParamType(myEPICSParamType),
            (int)myEPICSParamType);

  if (myEPICSParamType != asynParamNotDefined) {
    /* Some parameters are alread pre-created by the Controller.cpp,
       e.g.errorId. Use those, otherwise create a parameter */
    status = findParam(paramName, &function);
    if (status == asynSuccess) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s exist function=%d paramName=%s\n", modNamEMC,
                functionName, function, paramName);
    } else {
      status = createParam(paramName, myEPICSParamType, &function);
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s(%u) numPilsAsynDevInfo=%d created function=%d "
                "paramName=%s status=%s (%d)\n",
                modNamEMC, functionName, axisNo, numPilsAsynDevInfo, function,
                paramName, ethercatmcstrStatus(status), (int)status);
      if (status != asynSuccess) return -1;
    }
  }
  if (paramDescField) {
    char descName[64];
    asynStatus status;
    snprintf(descName, sizeof(descName), "%s_DESC", paramName);
    status =
        ethercatmcCreateParam(descName, asynParamOctet, &functionDescField);
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
              "%s%s(%u) paramDescField descName='%s' functionDescField=%d  "
              "status=%s (%d)\n",
              modNamEMC, functionName, axisNo, descName, functionDescField,
              ethercatmcstrStatus(status), (int)status);
    if (status == asynSuccess) {
      setStringParam(axisNo, functionDescField, paramDescField);
      setAlarmStatusSeverityWrapper(axisNo, functionDescField, asynSuccess);
    }
  }

  pPilsAsynDevInfo->axisNo = axisNo;
  pPilsAsynDevInfo->functionDescField = functionDescField;
  pPilsAsynDevInfo->functionNamAux0 = functionNamAux0;
  pPilsAsynDevInfo->functionStatusBits = functionStatusBits;
  pPilsAsynDevInfo->lenInPLC = lenInPLC;
  pPilsAsynDevInfo->inputOffset = inputOffset;
  pPilsAsynDevInfo->outputOffset = outputOffset;
  pPilsAsynDevInfo->statusOffset = statusOffset;
  pPilsAsynDevInfo->myEPICSParamType = myEPICSParamType;
  pPilsAsynDevInfo->iTypCode = iTypCode;
  pPilsAsynDevInfo->function = function;
  if (!strcmp(paramName, "SystemUTCtime")) {
    ctrlLocal.systemUTCtimePTPFunction = function;
    setAlarmStatusSeverityWrapper(0, defAsynPara.ethercatmcPTPdiffTimeIOC_MCU_,
                                  asynSuccess);
  } else if (!strcmp(paramName, "NTtimePackedTimeStructBias")) {
    ctrlLocal.systemNTtimePackedTimeStructBiasFunction = function;
    ctrlLocal.systemNTtimePackedTimeStructBiasFunctionStatusBits =
        functionStatusBits;
    int indexWr = 0;
    int functionWr = defAsynPara.ethercatmcPTPdiffXYtime_MCU_;
    setAlarmStatusSeverityWrapper(indexWr, functionWr, asynSuccess);
  } else if (!strcmp(paramName, "TcUTCtime")) {
    ctrlLocal.systemTcUTCtimeFunction = function;
    int indexWr = 1;
    int functionWr = defAsynPara.ethercatmcPTPdiffXYtime_MCU_;
    setAlarmStatusSeverityWrapper(indexWr, functionWr, asynSuccess);
  } else if (!strcmp(paramName, "TcNTPExttime")) {
    ctrlLocal.systemTcNTPExtTimeFunction = function;
    int indexWr = 2;
    int functionWr = defAsynPara.ethercatmcPTPdiffXYtime_MCU_;
    setAlarmStatusSeverityWrapper(indexWr, functionWr, asynSuccess);
  }
  if (!statusOffset) {
    /* The device has no status, so assume that the status is OK,
       expecially after a connection loss causing a
       setAlarmStatusSeverityAllReadbacks(asynDisconnected);
       and now we have the connection up again */
    setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);
  }
  /* Last action of this code: Increment the counter */
  ctrlLocal.numPilsAsynDevInfo = 1 + numPilsAsynDevInfo;
  return function;
}

int ethercatmcController::newPilsAsynDevice(int axisNo, unsigned devNum,
                                            unsigned indexOffset,
                                            unsigned iTypCode,
                                            unsigned iAllFlags,
                                            const char *paramName) {
  const static char *const functionName = "newPilsAsynDevice";
  unsigned numPilsAsynDevInfo = ctrlLocal.numPilsAsynDevInfo;
  unsigned lenInPLC = 0;
  unsigned inputOffset = 0;
  unsigned outputOffset = 0;
  unsigned statusOffset = 0;
  int functionNamAux0 = 0;
  int functionStatusBits = 0;
  asynParamType myAsynParamType = asynParamNotDefined;
  struct {
    char name[80]; /* 34 + some spare */
    unsigned axisNoOrIndex;
    char desc[80];
  } splitedParamNameNumberDesc;

  if ((strlen(paramName) < sizeof(splitedParamNameNumberDesc.name)) &&
      (strlen(paramName) < sizeof(splitedParamNameNumberDesc.desc))) {
    /* Need to split the parameter, like "EPOCHEL1252P#1" */
    int nvals;
    memset(&splitedParamNameNumberDesc, 0, sizeof(splitedParamNameNumberDesc));
    /* Try the long form: Temp#1#Temp of chassis */
    nvals =
        sscanf(paramName, "%[^#]#%u#%[^#]", &splitedParamNameNumberDesc.name[0],
               &splitedParamNameNumberDesc.axisNoOrIndex,
               &splitedParamNameNumberDesc.desc[0]);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s axisNo=%d iTypCode=0x%04X nvals=%d name='%s' "
              "axisNoOrIndex=%u desc='%s'\n",
              modNamEMC, functionName, axisNo, iTypCode, nvals,
              &splitedParamNameNumberDesc.name[0],
              splitedParamNameNumberDesc.axisNoOrIndex,
              &splitedParamNameNumberDesc.desc[0]);
    if (nvals >= 2) {
      int newAxisNo = (int)splitedParamNameNumberDesc.axisNoOrIndex;
      paramName = &splitedParamNameNumberDesc.name[0];
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
      inputOffset =
          indexOffset + lenInPLC;  // Look at the target value for readback
      outputOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1604:
      lenInPLC = 4;
      /* 1604 has "current value, followed by target value */
      inputOffset =
          indexOffset + lenInPLC;  // Look at the target value for readback
      outputOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamInt32;
      break;
    case 0x1802:
      /* 1802 has only a 32 bit status word */
      statusOffset = indexOffset;
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
    case 0x1A08:
      lenInPLC = 8;
      /* 1A08 has "current value, followed by extended status word; errorID is
       * ignored */
      inputOffset = indexOffset;
      statusOffset = indexOffset + lenInPLC;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
      myAsynParamType = asynParamInt64;
#endif
      break;
    case 0x1B04:
      lenInPLC = 4;
      /* 1B04 has "current value, followed by extended status word */
      inputOffset = indexOffset;
      statusOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamFloat64;
      break;
    case 0x1B08:
      lenInPLC = 8;
      /* 1B08 has "current value, followed by extended status word; errorID is
       * ignored */
      inputOffset = indexOffset;
      statusOffset = indexOffset + lenInPLC;
      myAsynParamType = asynParamFloat64;
      break;
  }
  /* Aux bits */
  if (iAllFlags & 0x03FFFFFF) {
    unsigned i;
    for (i = 0; i < MAX_REASON_AUX_BIT_SHOW; i++) {
      int function;
      asynStatus status;
      char auxBitname[64];
      snprintf(auxBitname, sizeof(auxBitname), "%s_NamAuxBit%u", paramName, i);
      status = findParam(auxBitname, &function);
      if (status == asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s exist function=%d\n", modNamEMC, auxBitname, function);
      } else {
        status = createParam(auxBitname, asynParamOctet, &function);
      }
      if (status == asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s(%u) numPilsAsynDevInfo=%d created function=%d "
                  "auxBitname=%s\n",
                  modNamEMC, auxBitname, axisNo, numPilsAsynDevInfo, function,
                  auxBitname);
        if (i == 0) functionNamAux0 = function;
      } else {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s(%u) numPilsAsynDevInfo=%d not created function=%d "
                  "auxBitname=%s status=%s (%d)\n",
                  modNamEMC, auxBitname, axisNo, numPilsAsynDevInfo, function,
                  auxBitname, ethercatmcstrStatus(status), (int)status);
      }
    }
    if (ctrlLocal.supported.bPILSv2) {
      newIndexerAxisAuxBitsV2(NULL, /* pAxis */
                              axisNo, devNum, iAllFlags, functionNamAux0,
                              0.0,  // fAbsMin,
                              0.0,  // fAbsMax,
                              indexOffset);
    }
  }
  /* Status word */
  if (statusOffset) {
    int function;
    asynStatus status;
    char statusBitsName[64];
    if (inputOffset) {
      snprintf(statusBitsName, sizeof(statusBitsName), "%s_StatusBits",
               paramName);
    } else {
      /* probably an 1802 which is only a status word: use the name as is */
      snprintf(statusBitsName, sizeof(statusBitsName), "%s", paramName);
    }
    status = findParam(statusBitsName, &function);
    if (status == asynSuccess) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s exist function=%d\n", modNamEMC, statusBitsName,
                function);
    } else {
      status = createParam(statusBitsName, asynParamUInt32Digital, &function);
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s(%u) numPilsAsynDevInfo=%d created function=%d "
                "statusBitsName=%s status=%s (%d)\n",
                modNamEMC, statusBitsName, axisNo, numPilsAsynDevInfo, function,
                statusBitsName, ethercatmcstrStatus(status), (int)status);
    }
    if (status == asynSuccess) functionStatusBits = function;
  }

  if (!lenInPLC && !statusOffset) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s(%u) iTypCode not supported, ignored 0x%X\n", modNamEMC,
              functionName, numPilsAsynDevInfo, iTypCode);
    return -1;
  }
  if ((myAsynParamType == asynParamNotDefined) && lenInPLC) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s(%u) pilsNo=%d not created paramName=%s\n", modNamEMC,
              functionName, axisNo, numPilsAsynDevInfo, paramName);
    return -1;
  }
  return addPilsAsynDevLst(
      axisNo, paramName,
      splitedParamNameNumberDesc.desc[0] ? &splitedParamNameNumberDesc.desc[0]
                                         : NULL,
      functionNamAux0, functionStatusBits, lenInPLC, inputOffset, outputOffset,
      statusOffset, myAsynParamType, iTypCode);
}

extern "C" void UTCtimeToEpicsTimeStamp(uint64_t dcNsec, epicsTimeStamp *ts) {
#define NSEC_PER_SEC 1000000000
  /*
   * convert from UTC Time to Epics time
   * (POSIX_TIME_AT_EPICS_EPOCH defined in epicsTime.h)
   */
  uint64_t nSecEpicsEpoch;
  nSecEpicsEpoch =
      dcNsec - ((uint64_t)POSIX_TIME_AT_EPICS_EPOCH) * NSEC_PER_SEC;
  ts->secPastEpoch = (uint32_t)(nSecEpicsEpoch / NSEC_PER_SEC);
  ts->nsec = (uint32_t)(nSecEpicsEpoch % NSEC_PER_SEC);
}

/*
 * Find if an asyn parameter is connected to an indexer output
 */
pilsAsynDevInfo_type *ethercatmcController::findIndexerOutputDevice(
    int axisNo, int function, asynParamType myEPICSParamType) {
  for (unsigned numPilsAsynDevInfo = 0;
       numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
       numPilsAsynDevInfo++) {
    pilsAsynDevInfo_type *pPilsAsynDevInfo =
        &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

    if ((axisNo == pPilsAsynDevInfo->axisNo) &&
        (function == pPilsAsynDevInfo->function) &&
        (myEPICSParamType == pPilsAsynDevInfo->myEPICSParamType) &&
        (pPilsAsynDevInfo->outputOffset))
      return pPilsAsynDevInfo;
  }
  return NULL;
}

void ethercatmcController::changedAuxBits_to_ASCII(
    int axisNo, int functionNamAux0, epicsUInt32 statusReasonAux,
    epicsUInt32 oldStatusReasonAux) {
  /* Show even bit 27..24, which are reson bits, here */
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
        status = getStringParam(axisNo, function, (int)length,
                                &ctrlLocal.changedAuxBits[auxBitIdx][1]);
      }
      if (status != asynSuccess) {
        snprintf(&ctrlLocal.changedAuxBits[auxBitIdx][1], length, "Bit%d",
                 auxBitIdx);
      }
      if ((statusReasonAux >> auxBitIdx) & 0x01) {
        ctrlLocal.changedAuxBits[auxBitIdx][0] = '+';
      } else {
        ctrlLocal.changedAuxBits[auxBitIdx][0] = '-';
      }
    }
  }
}

asynStatus ethercatmcController::indexerPoll(void) {
  if (ctrlLocal.pIndexerProcessImage && ctrlLocal.lastDeviceEndOffset) {
    size_t indexOffset = ctrlLocal.firstDeviceStartOffset;
    size_t len = ctrlLocal.lastDeviceEndOffset - indexOffset;
    asynStatus status;
    int traceMask = ASYN_TRACEIO_DRIVER;
    memset(ctrlLocal.pIndexerProcessImage, 0, ctrlLocal.lastDeviceEndOffset);
    status = getPlcMemoryOnErrorStateChange(
        indexOffset, &ctrlLocal.pIndexerProcessImage[indexOffset], len);
    if (status) traceMask |= ASYN_TRACE_ERROR;
    asynPrint(pasynUserController_, traceMask,
              "%spoll() indexOffset=%u len=%u status=%s (%d)\n", modNamEMC,
              (unsigned)indexOffset, (unsigned)len, ethercatmcstrStatus(status),
              (int)status);
    if (status) return status;

    {
      /* Extract devices, which are not motors */
      for (unsigned numPilsAsynDevInfo = 0;
           numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
           numPilsAsynDevInfo++) {
        pilsAsynDevInfo_type *pPilsAsynDevInfo =
            &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];

        unsigned inputOffset = pPilsAsynDevInfo->inputOffset;
        unsigned statusOffset = pPilsAsynDevInfo->statusOffset;
        int axisNo = pPilsAsynDevInfo->axisNo;
        const char *paramName = "";
        int function = pPilsAsynDevInfo->function;
        int functionStatusBits = pPilsAsynDevInfo->functionStatusBits;
        void *pDataInPlc = &ctrlLocal.pIndexerProcessImage[inputOffset];
        if (function) {
          getParamName(axisNo, function, &paramName);
        } else if (functionStatusBits) {
          getParamName(axisNo, functionStatusBits, &paramName);
        }
        asynPrint(pasynUserController_, ASYN_TRACE_FLOW,
                  "%sindexerPoll(%d) numPilsAsynDevInfo=%u '%s' inputOffset=%u "
                  "statusOffset=%u functionStatusBits=%d\n",
                  modNamEMC, axisNo, numPilsAsynDevInfo, paramName, inputOffset,
                  statusOffset, functionStatusBits);
        if (statusOffset) {
          /* Add a printout for the changed AUX bits.
             currently the poller for the axis has simiar code */
          const static epicsUInt32 maskStatusReasonAux = 0x03FFFFFF;
          void *pStatusInPlc = &ctrlLocal.pIndexerProcessImage[statusOffset];
          epicsUInt32 statusReasonAux =
              netToUint(pStatusInPlc, sizeof(statusReasonAux));
          if (functionStatusBits) {
            int functionNamAux0 = pPilsAsynDevInfo->functionNamAux0;
            if (functionNamAux0) {
              epicsUInt32 oldStatusReasonAux;
              getUIntDigitalParam(axisNo, functionStatusBits,
                                  &oldStatusReasonAux, 0xFFFFFFFF);
              if ((statusReasonAux ^ oldStatusReasonAux) &
                  maskStatusReasonAux) {
                changedAuxBits_to_ASCII(axisNo, functionNamAux0,
                                        statusReasonAux, oldStatusReasonAux);
                asynPrint(
                    pasynUserController_, traceMask | ASYN_TRACE_INFO,
                    "%spoll(%d) %sOld=0x%04X new=0x%04X "
                    "(%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s)"
                    "\n",
                    modNamEMC, axisNo, paramName, oldStatusReasonAux,
                    statusReasonAux, ctrlLocal.changedAuxBits[27],
                    ctrlLocal.changedAuxBits[26], ctrlLocal.changedAuxBits[25],
                    ctrlLocal.changedAuxBits[24], ctrlLocal.changedAuxBits[0],
                    ctrlLocal.changedAuxBits[1], ctrlLocal.changedAuxBits[2],
                    ctrlLocal.changedAuxBits[3], ctrlLocal.changedAuxBits[4],
                    ctrlLocal.changedAuxBits[5], ctrlLocal.changedAuxBits[6],
                    ctrlLocal.changedAuxBits[7], ctrlLocal.changedAuxBits[8],
                    ctrlLocal.changedAuxBits[9], ctrlLocal.changedAuxBits[10],
                    ctrlLocal.changedAuxBits[11], ctrlLocal.changedAuxBits[12],
                    ctrlLocal.changedAuxBits[13], ctrlLocal.changedAuxBits[14],
                    ctrlLocal.changedAuxBits[15], ctrlLocal.changedAuxBits[16],
                    ctrlLocal.changedAuxBits[17], ctrlLocal.changedAuxBits[18],
                    ctrlLocal.changedAuxBits[19], ctrlLocal.changedAuxBits[20],
                    ctrlLocal.changedAuxBits[21], ctrlLocal.changedAuxBits[22],
                    ctrlLocal.changedAuxBits[23]);
              }
            }
            setUIntDigitalParam(axisNo, functionStatusBits,
                                (epicsUInt32)statusReasonAux, 0xFFFFFFFF);
          }
          setAlarmStatusSeverityFromStatusBits(axisNo, function,
                                               statusReasonAux);
        }
        if (!inputOffset) continue;

        /* Each axis has it's own parameters.
           axisNo == 0 is no special axis, parameters
           for "the controller", additional IO, PTP info */
        switch (pPilsAsynDevInfo->myEPICSParamType) {
          case asynParamInt32: {
            int tracelevel = ASYN_TRACE_FLOW;
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            epicsInt32 newValue, oldValue;
            newValue = (epicsInt32)netToSint(pDataInPlc, lenInPLC);
            status = getIntegerParam(axisNo, function, &oldValue);
            if (status != asynSuccess || oldValue != newValue) {
              if (pPilsAsynDevInfo->outputOffset) tracelevel |= ASYN_TRACE_INFO;
              status = setIntegerParam(axisNo, function, newValue);
              if (status == asynParamWrongType) {
                asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                          "%sindexerPoll(%d) ERROR: need to disable "
                          "function=%s(%d) status=%s (%d)\n",
                          modNamEMC, axisNo, paramName, function,
                          ethercatmcstrStatus(status), (int)status);
                pPilsAsynDevInfo->inputOffset = 0;
              } else {
                ctrlLocal.callBackNeeded |= 1 << axisNo;
                if (status != asynSuccess) tracelevel |= ASYN_TRACE_INFO;
              }
            }
            asynPrint(
                pasynUserController_, tracelevel,
                "%sindexerPoll(%d) function=%s(%d) oldValue=%d newValue=%d\n",
                modNamEMC, axisNo, paramName, function, oldValue, newValue);
          } break;
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
                  newValue =
                      (double)(epicsInt64)netToSint(pDataInPlc, lenInPLC);
                  break;
                case 0x1204:
                case 0x1A08:
                  newValue =
                      (double)(epicsInt64)netToSint64(pDataInPlc, lenInPLC);
                  break;
                case 0x1B04:
                case 0x1B08:
                  newValue = (double)netToDouble(pDataInPlc, lenInPLC);
                  break;
                default:
                  asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                            "%sindexerPoll(%d) ERROR: newValueValid = 0 "
                            "function=%s(%d)\n",
                            modNamEMC, axisNo, paramName, function);
                  newValueValid = 0;
              }
              getParamName(axisNo, function, &paramName);
              if (newValueValid) {
                status = getDoubleParam(axisNo, function, &oldValue);
                if (status != asynSuccess || oldValue != newValue) {
                  status = setDoubleParam(axisNo, function, newValue);
                  if (status == asynParamWrongType) {
                    asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                              "%sindexerPoll(%d) ERROR: need to disable "
                              "function=%s(%d) status=%s (%d)\n",
                              modNamEMC, axisNo, paramName, function,
                              ethercatmcstrStatus(status), (int)status);
                    pPilsAsynDevInfo->inputOffset = 0;
                  } else {
                    ctrlLocal.callBackNeeded |= 1 << axisNo;
                    if (status != asynSuccess) tracelevel |= ASYN_TRACE_INFO;
                  }
                }
              }
              asynPrint(
                  pasynUserController_, tracelevel,
                  "%sindexerPoll(%d) function=%s(%d) oldValue=%f newValue=%f\n",
                  modNamEMC, axisNo, paramName, function, oldValue, newValue);
            }
            break;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
          case asynParamInt64: {
            int tracelevel = ASYN_TRACE_FLOW;
            const char *paramName = "";
            unsigned lenInPLC = pPilsAsynDevInfo->lenInPLC;
            epicsInt64 newValue, oldValue;
            getParamName(axisNo, function, &paramName);
            newValue = (epicsInt64)netToSint64(pDataInPlc, lenInPLC);
            status = getInteger64Param(axisNo, function, &oldValue);
            if (status != asynSuccess || oldValue != newValue) {
              status = setInteger64Param(axisNo, function, newValue);
              if (status == asynParamWrongType) {
                asynPrint(pasynUserController_, ASYN_TRACE_ERROR,
                          "%sindexerPoll(%d) ERROR: need to disable "
                          "function=%s(%d) status=%s (%d)\n",
                          modNamEMC, axisNo, paramName, function,
                          ethercatmcstrStatus(status), (int)status);
                pPilsAsynDevInfo->inputOffset = 0;
              } else {
                ctrlLocal.callBackNeeded |= 1 << axisNo;
                if (status != asynSuccess) tracelevel |= ASYN_TRACE_INFO;
              }
            }
            asynPrint(pasynUserController_, tracelevel,
                      "%sindexerPoll(%d) function=%s(%d)  oldValue=%" PRIi64
                      " newValue=%" PRIi64 "\n",
                      modNamEMC, axisNo, paramName, function, (int64_t)oldValue,
                      (int64_t)newValue);
          } break;
#endif
          default:;
        }
      } /* for */
      {
        /* Special code for timing, PTP, NTP */
        int function = ctrlLocal.systemUTCtimePTPFunction;
        if (function) {
          /* PTP on the MCU does exist */
          epicsTimeStamp timePTP_MCU;
          memset(&timePTP_MCU, 0, sizeof(timePTP_MCU));
          indexerSystemUTCtime(function, &timePTP_MCU);
          /* because PTP does exist, check it against NTP in the IOC */
          function = ctrlLocal.systemNTtimePackedTimeStructBiasFunction;
          if (function) {
            indexerNTtimePackedTimeStructBias(
                function,
                ctrlLocal.systemNTtimePackedTimeStructBiasFunctionStatusBits,
                &timePTP_MCU);
          }
          /* See if the TcTime exists. If yes, diff against PTP */
          if (ctrlLocal.systemTcUTCtimeFunction) {
            int indexRd = 0;
            int indexWr = 1;
            indexerPTPdiffXYtime(ctrlLocal.systemTcUTCtimeFunction, indexRd,
                                 indexWr, &timePTP_MCU);
          }
          if (ctrlLocal.systemTcNTPExtTimeFunction) {
            int indexRd = 0;
            int indexWr = 2;
            indexerPTPdiffXYtime(ctrlLocal.systemTcNTPExtTimeFunction, indexRd,
                                 indexWr, &timePTP_MCU);
          }
        }
      }
    }
    return status;
  }
  return asynDisabled;
}

void ethercatmcController::indexerSystemUTCtime(int function,
                                                epicsTimeStamp *pTimePTP_MCU) {
  int indexRd = 0;
  epicsInt64 oldValue = 0;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
  getInteger64Param(indexRd, function, &oldValue);
#endif
  uint64_t nSec = (uint64_t)oldValue;
  epicsTimeStamp timeIOC;
  UTCtimeToEpicsTimeStamp(nSec, pTimePTP_MCU);
  asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
            "%sindexerPoll SystemUTCtime nSec=%" PRIu64 " sec:nSec=%09u.%09u\n",
            modNamEMC, nSec, pTimePTP_MCU->secPastEpoch, pTimePTP_MCU->nsec);
  setTimeStamp(pTimePTP_MCU);
  ctrlLocal.callBackNeeded |= 1 << indexRd;
  int rtn = epicsTimeGetCurrent(&timeIOC);
  if (!rtn) {
    int function = defAsynPara.ethercatmcPTPdiffTimeIOC_MCU_;
    double time_IOC_ms = (((double)timeIOC.secPastEpoch) * 1000.0) +
                         (((double)timeIOC.nsec) / 1000000.0);
    double time_MCU_ms = (((double)pTimePTP_MCU->secPastEpoch) * 1000.0) +
                         (((double)pTimePTP_MCU->nsec) / 1000000.0);
    double diffTimeIOC_MCU = time_IOC_ms - time_MCU_ms;
    (void)setDoubleParam(indexRd, function, diffTimeIOC_MCU);
  } else {
    setAlarmStatusSeverityWrapper(indexRd, function, asynDisconnected);
  }
}

void ethercatmcController::indexerPTPdiffXYtime(
    int functionRd, int indexRd, int indexWr,
    const epicsTimeStamp *pTimePTP_MCU) {
  epicsInt64 oldValue = 0;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
  getInteger64Param(indexRd, functionRd, &oldValue);
#endif

  uint64_t nSec = (uint64_t)oldValue;
  epicsTimeStamp timeTcUTC_MCU;
  UTCtimeToEpicsTimeStamp(nSec, &timeTcUTC_MCU);

  double msecPTPdiffTctime_MCU =
      1000.0 * epicsTimeDiffInSeconds(&timeTcUTC_MCU, pTimePTP_MCU);
  asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
            "%sindexerPoll timeTcUTC_MCU sec:nSec=%010u.%09u "
            "msecPTPdiffTctime_MCU=%f\n",
            modNamEMC, timeTcUTC_MCU.secPastEpoch, timeTcUTC_MCU.nsec,
            msecPTPdiffTctime_MCU);
  {
    int functionWr = defAsynPara.ethercatmcPTPdiffXYtime_MCU_;
    (void)setDoubleParam(indexWr, functionWr, msecPTPdiffTctime_MCU);
  }
}

void ethercatmcController::indexerNTtimePackedTimeStructBias(
    int function, int functionStatusBits, const epicsTimeStamp *pTimePTP_MCU) {
  int indexRd = 0;
  idxStatusCodeType idxStatusCode = idxStatusCodeIDLE;  // good case
  if (functionStatusBits) {
    epicsUInt32 statusReasonAux = 0;
    getUIntDigitalParam(indexRd, functionStatusBits, &statusReasonAux,
                        0xFFFFFFFF);
    idxStatusCode = (idxStatusCodeType)(statusReasonAux >> 28);
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
              "%sindexerPoll systemNTtimePackedTimeStructFunction "
              "statusReasonAux=0x%x idxStatusCode=%x functionStatusBits=%d\n",
              modNamEMC, statusReasonAux, (unsigned)idxStatusCode,
              functionStatusBits);
  }
  if (idxStatusCode == idxStatusCodeIDLE) {
    epicsInt64 oldValue = 0;
#ifdef ETHERCATMC_ASYN_ASYNPARAMINT64
    getInteger64Param(indexRd, function, &oldValue);
#endif
    uint64_t packedTimeStruct = (uint64_t)oldValue;
    struct tm tm;
    epicsTimeStamp NTtime_MCU;
    unsigned millisec;
    int32_t mcu_bias_hours, bias_in_seconds;
    memset(&tm, 0, sizeof(tm));
    millisec = packedTimeStruct & 0x3FF;
    packedTimeStruct >>= 10;
    tm.tm_sec = packedTimeStruct & 0x3F;
    packedTimeStruct >>= 6;
    tm.tm_min = packedTimeStruct & 0x3F;
    packedTimeStruct >>= 6;
    tm.tm_hour = packedTimeStruct & 0x1F;
    packedTimeStruct >>= 5;
    tm.tm_mday = packedTimeStruct & 0x1F;
    packedTimeStruct >>= 5;
    /* day is 1..31; month is 0..11 */
    tm.tm_mon = (packedTimeStruct & 0x0F) - 1;
    packedTimeStruct >>= 4;
    /* MCU sends year without century (=2000), epicsTimeFromTM() wants year -
     * 1900 */
    tm.tm_year = 100 + (packedTimeStruct & 0x7F);
    packedTimeStruct >>= 7;
    mcu_bias_hours = packedTimeStruct & 0x1F;
    /* sign extend the bias, minutes */
    bias_in_seconds =
        (mcu_bias_hours & 0x1F) ? mcu_bias_hours | 0xFFFFFFF0 : mcu_bias_hours;
    bias_in_seconds *= 3600; /* hours -> seconds */

    time_t unix_epoch = timegm(&tm);
    unix_epoch += bias_in_seconds;
    NTtime_MCU.secPastEpoch = unix_epoch - POSIX_TIME_AT_EPICS_EPOCH;
    NTtime_MCU.nsec = millisec * 1000000;
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
              "%sindexerPoll tm=%04u-%02u-%02u %02u:%02u:%02u.%03u NTtime_MCU "
              "unix_epoch=%lu sec:nSec=%010u.%09u\n",
              modNamEMC, tm.tm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour,
              tm.tm_min, tm.tm_sec, millisec, (unsigned long)unix_epoch,
              NTtime_MCU.secPastEpoch, NTtime_MCU.nsec);
    indexerCalcPTPdiffXYtime_MCU(indexRd,
                                 defAsynPara.ethercatmcPTPdiffXYtime_MCU_,
                                 &NTtime_MCU, pTimePTP_MCU);
  }
}

void ethercatmcController::indexerCalcPTPdiffXYtime_MCU(
    int indexRd, int function, const epicsTimeStamp *pNTtime_MCU,
    const epicsTimeStamp *pTimePTP_MCU) {
#if 0
  {
    char buf[40];
    epicsTimeToStrftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                        pTimePTP_MCU);
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW | ASYN_TRACE_INFO,
              "%sindexerPoll %s ZZ timPTP_MCU UNIX=%lu EPICS=sec:nSec=%010u.%09u\n",
              modNamEMC, buf,
              (unsigned long)pTimePTP_MCU->secPastEpoch + POSIX_TIME_AT_EPICS_EPOCH,
              pTimePTP_MCU->secPastEpoch, pTimePTP_MCU->nsec);
    epicsTimeToStrftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
                        pNTtime_MCU);
    asynPrint(pasynUserController_, ASYN_TRACE_FLOW | ASYN_TRACE_INFO,
              "%sindexerPoll %s ZZ NTtime_MCU UNIX=%lu EPICS=sec:nSec=%010u.%09u\n",
              modNamEMC, buf,
              (unsigned long)pNTtime_MCU->secPastEpoch + POSIX_TIME_AT_EPICS_EPOCH,
              pNTtime_MCU->secPastEpoch, pNTtime_MCU->nsec);
  }
#endif
  double msecPTPdiffXYtime_MCU =
      1000.0 * epicsTimeDiffInSeconds(pNTtime_MCU, pTimePTP_MCU);
  asynPrint(
      pasynUserController_, ASYN_TRACE_FLOW /* | ASYN_TRACE_INFO */,
      "%sindexerPoll NTtime_MCU sec:nSec=%010u.%09u msecPTPdiffXYtime_MCU=%f\n",
      modNamEMC, pNTtime_MCU->secPastEpoch, pNTtime_MCU->nsec,
      msecPTPdiffXYtime_MCU);
  (void)setDoubleParam(indexRd, function, msecPTPdiffXYtime_MCU);
}

void ethercatmcController::indexerDisconnected(void) {
  if (!ctrlLocal.oldStatus || ctrlLocal.numPilsAsynDevInfo) {
    asynPrint(pasynUserController_, ASYN_TRACE_INFO, "%s%s\n", modNamEMC,
              "indexerDisconnected");
  }
  for (int axisNo = 0; axisNo < numAxes_; axisNo++) {
    setIntegerParam(axisNo, motorStatusGainSupport_, 0);
    setStringParam(axisNo, defAsynPara.ethercatmcErrTxt_, "E: Communication");
  }

  if (ctrlLocal.numPilsAsynDevInfo) {
    for (unsigned numPilsAsynDevInfo = 0;
         numPilsAsynDevInfo < ctrlLocal.numPilsAsynDevInfo;
         numPilsAsynDevInfo++) {
      pilsAsynDevInfo_type *pPilsAsynDevInfo =
          &ctrlLocal.pilsAsynDevInfo[numPilsAsynDevInfo];
      setAlarmStatusSeverityWrapper(pPilsAsynDevInfo->axisNo,
                                    pPilsAsynDevInfo->function,
                                    asynDisconnected);

      {
        int axisNo = pPilsAsynDevInfo->axisNo;
        int function = pPilsAsynDevInfo->function;
        const char *paramName = NULL;
        if (!(getParamName(function, &paramName))) {
          int functionDescField = 0;
          char descName[64];
          snprintf(descName, sizeof(descName), "%s_DESC", paramName);
          if (!findParam(descName, &functionDescField)) {
            setStringParam(axisNo, functionDescField, "");
            setAlarmStatusSeverityWrapper(axisNo, functionDescField,
                                          asynDisconnected);
          }
        }
      }
    }
    memset(&ctrlLocal.pilsAsynDevInfo, 0, sizeof(ctrlLocal.pilsAsynDevInfo));
    memset(&ctrlLocal.customParaName, 0, sizeof(ctrlLocal.customParaName));
    memset(&ctrlLocal.functionFromParamIndex, 0,
           sizeof(ctrlLocal.functionFromParamIndex));
    ctrlLocal.numPilsAsynDevInfo = 0;
  }
  free(ctrlLocal.pIndexerProcessImage);
  ctrlLocal.pIndexerProcessImage = NULL;
}
