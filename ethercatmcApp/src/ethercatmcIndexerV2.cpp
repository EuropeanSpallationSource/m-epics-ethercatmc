/*
  FILENAME... ethercatmcIndexerV3.cpp
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

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#define MAX_COUNTER 14

/* Re-define calles without FILE and LINE */
#define getPlcMemoryOnErrorStateChange(a,b,c)  getPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)
#define setPlcMemoryOnErrorStateChange(a,b,c)  setPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)

/* No "direct" calls into ADS below this point */
#undef getPlcMemoryViaADS
#undef setPlcMemoryViaADS

static const double fABSMIN = -3.0e+38;
static const double fABSMAX =  3.0e+38;

extern "C" {
  const char *plcUnitTxtFromUnitCodeV2(unsigned unitCode)
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
  const char *plcUnitPrefixTxtV2(int prefixCode)
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

asynStatus ethercatmcController::indexerInitialPollv2(void)
{
  asynStatus status;
  unsigned firstDeviceStartOffset = (unsigned)-1; /* Will be decreased while we go */
  unsigned lastDeviceEndOffset = 0;  /* will be increased while we go */
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
  ethercatmcIndexerAxis *pAxis = NULL;

  memset(&descVersAuthors, 0, sizeof(descVersAuthors));
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
      case 0x1802:
        /* The first axisNo goes to axis#0, which is not used for axes
           all others need their own axis numbers */
        if (axisNo) {
          axisNo++;
        }
        break;
      case 0x1E04:
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
              plcUnitPrefixTxtV2(( (int8_t)((iUnit & 0xFF00)>>8))),
              plcUnitTxtFromUnitCodeV2(iUnit & 0xFF),
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
    case 0x1802:
      {
        const char *paramName = descVersAuthors.desc;
        int function = newPilsAsynDevice(axisNo, iOffsBytes, iTypCode, paramName);
        if (function < 0) {
          status = asynError;
          goto endPollIndexer;
        }
        status = newIndexerAxisAuxBitsV2(NULL, /* pAxis */
                                         axisNo,
                                         devNum,
                                         iAllFlags,
                                         fAbsMin,
                                         fAbsMax,
                                         iOffsBytes);
      }
      break;
    case 0x1E04:
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
        status = newIndexerAxisAuxBitsV2(pAxis,
                                         pAxis->axisNo_,
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
                 plcUnitPrefixTxtV2(( (int8_t)((iUnit & 0xFF00)>>8))),
                 plcUnitTxtFromUnitCodeV2(iUnit & 0xFF));
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
                     plcUnitPrefixTxtV2(( (int8_t)((iUnit & 0xFF00)>>8))),
                     plcUnitTxtFromUnitCodeV2(iUnit & 0xFF));
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
  return status;
}

asynStatus
ethercatmcController::newIndexerAxisAuxBitsV2(ethercatmcIndexerAxis *pAxis,
                                              unsigned axisNo,
                                              unsigned devNum,
                                              unsigned iAllFlags,
                                              double   fAbsMin,
                                              double   fAbsMax,
                                              unsigned iOffsBytes)
{
  asynStatus status = asynSuccess;
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
          setStringParam(axisNo, function, auxBitName);
          setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);
        }
        if (!pAxis) {
          ; /* Do nothing */
        } else if (!strcmp("notHomed", auxBitName)) {
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
  if (pAxis) {
    udateMotorLimitsRO(axisNo,
                       (fAbsMin > fABSMIN && fAbsMax < fABSMAX),
                       fAbsMax,
                       fAbsMin);
  }
#endif

  return status;
}

asynStatus
ethercatmcController::indexerReadAxisParametersV2(ethercatmcIndexerAxis *pAxis,
                                                  unsigned devNum)
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
    unsigned bitIdx;

    for (bitIdx = 0; bitIdx <= 15; bitIdx++) {
      unsigned paramIndex = dataIdx*16 + bitIdx;
      if (paramIndex >= sizeof(pAxis->drvlocal.PILSparamPerm)) {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%sparameters(%d) paramIdx (%u 0x%02X) out of range\n",
                  modNamEMC, axisNo,
                  paramIndex, paramIndex);
        return asynError;
      }
      unsigned bitIsSet = parameters & (1 << bitIdx) ? 1 : 0;
      if (bitIsSet) {
        pAxis->drvlocal.PILSparamPerm[paramIndex] = PILSparamPermWrite;
        if (paramIndexIsIntegerV2(paramIndex)) {
          pAxis->drvlocal.lenInPlcParaInteger[paramIndex] = 4; /* sizeof(int32) */
        } else {
          pAxis->drvlocal.lenInPlcParaFloat[paramIndex] = 8; /* sizeof(double) */
        }
      }
    }
  }
  return asynSuccess;
}
