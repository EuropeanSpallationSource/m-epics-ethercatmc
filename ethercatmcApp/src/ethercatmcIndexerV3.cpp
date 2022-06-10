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

#include <epicsThread.h>

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#define MAX_COUNTER 14

static const double fABSMIN = -3.0e+38;
static const double fABSMAX =  3.0e+38;

extern "C" {
  const char *getPlcBaseUnitTxtFromUnitCodeV3(unsigned unitCode)
  {
    unsigned baseUnitCode = (unitCode & 0x3F);
    const static char *const baseUnitTxts[] = {
      "",
      NULL,
      "bar",
      "counts",
      "degree",
      "gr",
      "m",
      "m2",
      "m3",
      "sec",
      "Ampere",
      "Farad",
      "Henry",
      "°Kelvin",
      "Ohm",
      "Tesla",
      "Volt",
      "Watt",
      "°Celsius",
      "°Fahrenheit",
      "bit",
      "steps",
      "?"};
    if (baseUnitCode < sizeof(baseUnitTxts)/sizeof(baseUnitTxts[0]))
      return baseUnitTxts[baseUnitCode];

    return "?";
  }

  const char *getPlcTimeBaseTxtFromUnitCodeV3(unsigned unitCode)
  {
    unsigned timeBaseCode = (unitCode >> 6) & 0x1F;
    const static char *const timeBaseTxts[] = {
      "",
      NULL,
      "/sec",
      "/sec2",
      "/sec3",
      "/min",
      "/hour",
      "/day",
      "??" };
    if (timeBaseCode < sizeof(timeBaseTxts)/sizeof(timeBaseTxts[0]))
      return timeBaseTxts[timeBaseCode];

    return "??";
  }
  const char *getPlcUnitExponentTxtV3(unsigned unitCode)
  {
    unsigned exponentCode = (unitCode >> 11) & 0x1F;
    if (exponentCode < 0x10) {
      /* we have 5 bits. The highest one is the sign */
      switch (exponentCode) {
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
        return "????";
      }
    } else {
      switch (0x20 - exponentCode) {
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
        return "????";
      }
    }
  }
  void unitCodeToText(char *buf, size_t buflen, unsigned unitCode,
                      const char *mainBaseUnitTxt,
                      const char *mainUnitExponentTxt)
  {
    const char *plcBaseUnitTxt = getPlcBaseUnitTxtFromUnitCodeV3(unitCode);
    const char *plcTimeBaseTxt = getPlcTimeBaseTxtFromUnitCodeV3(unitCode);
    const char *plcUnitExponentTxt = getPlcUnitExponentTxtV3(unitCode);

    /* if plcBaseUnitTxt == NULL, take both BaseUnit and UnitExponent from main */
    snprintf(buf, buflen, "%s%s%s",
             plcBaseUnitTxt ? plcUnitExponentTxt : mainUnitExponentTxt,
             plcBaseUnitTxt ? plcBaseUnitTxt : mainBaseUnitTxt,
             plcTimeBaseTxt);
  }
};


/* Re-define calles without FILE and LINE */
#define getPlcMemoryOnErrorStateChange(a,b,c)  getPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)
#define setPlcMemoryOnErrorStateChange(a,b,c)  setPlcMemoryOnErrorStateChangeFL(a,b,c,__FILE__, __LINE__)

/* No "direct" calls into ADS below this point */
#undef getPlcMemoryViaADS
#undef setPlcMemoryViaADS

typedef union {
  struct {
    uint8_t descriptor_type_0x1010     [2];
    uint8_t last_descriptor_id         [2];
    uint8_t plc_description_id         [2];
    uint8_t plc_version_descriptor_id  [2];
    uint8_t plc_author_descriptor_id   [2];
    uint8_t descriptor_slot_size       [2];
    uint8_t number_of_devices          [2];
    uint8_t plc_flags                  [2];
    char    name_of_plc                [56];
  } plcDescriptor;

  struct {
    uint8_t descriptor_type_0x2014                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t string_description_id                    [2];
    uint8_t target_param_descriptor_id               [2];
    uint8_t auxbits_bitfield_flag_descriptor_id      [2];
    uint8_t parameters_descriptor_id                 [2];
    uint8_t enum_errorId_descriptor_id               [2];
    uint8_t type_code                                [2];
    uint8_t device_offset                            [2];
    uint8_t device_flags                             [2];
    char    device_name                              [64];
  } deviceDescriptor;

  struct {
    uint8_t descriptor_type_0x3004     [2];
    uint8_t prev_descriptor_id         [2];
    char    utf8_string                [64];
  } stringDescriptor;

  struct {
    uint8_t descriptor_type_0x4006                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t enum_value                               [2];
    char    enum_name                                [64];
  } enumDescriptor;

  struct {
    uint8_t descriptor_type_0x5008                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t last_descriptor_id                       [2];
    uint8_t lowest_bit                               [1];
    uint8_t bit_width                                [1];
    char    bitfield_name                            [64];
  } bitfieldDescriptor;

  struct {
    uint8_t descriptor_type_0x5105                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t bit_number                               [1]; /* 1 byte only */
    char    flag_name                                [64];
  } flagDescriptor;

  struct {
    uint8_t descriptor_type_0x6114                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t string_description_id                    [2];
    uint8_t parameterIndex                           [2];
    uint8_t parameterTypV3                           [2];
    uint8_t unit                                     [2];
    uint8_t min_value                                [4];
    uint8_t max_value                                [4];
    char    parameter_name                           [64];
  } parameterDescriptor;

  struct {
    uint8_t descriptor_type_0x620e                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t string_description_id                    [2];
    uint8_t enumparam_read_id                        [2];
    uint8_t enumparam_write_id                       [2];
    uint8_t enumparam_index                          [2];
    uint8_t enumparam_type                           [2];
    char    enumparam_name                           [64];
  } enumparamDescriptor;

  struct {
    uint8_t descriptor_type_0x680e                   [2];
    uint8_t prev_descriptor_id                       [2];
    uint8_t string_description_id                    [2];
    uint8_t function_argument_id                     [2];
    uint8_t function_result_id                       [2];
    uint8_t function_index                           [2];
    uint8_t function_flags                           [2];
    char    function_name                            [64];
  } functionDescriptor;

  struct {
    uint8_t descriptor_type_0xFFFF     [2];
    uint8_t cycleCounter               [2];
    uint8_t lengthOfDebugDescriptor    [2];
    char    message                    [58];
  } debugDescriptor;

  struct {
    uint8_t descriptor_type          [2];
    uint8_t descriptor_prev          [2];
  } genericDescriptor;
} allDescriptors_type;

asynStatus ethercatmcController::readMailboxV3FL(unsigned descID,
                                                 void *bufptr, size_t buflen,
                                                 const char *fileName,
                                                 int lineNo)
{
  static const char * const c_function_name = "readMailboxV3";
  asynStatus status;
  unsigned value = descID;
  unsigned valueAcked = 0x8000 + value;
  unsigned counter = 0;
  if (descID > 0x7FFF) {
    status = asynDisabled;
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%s%s descID=%d status=%s (%d)\n",
              modNamEMC, c_function_name, descID,
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
              "%s%s status=%s (%d)\n",
              modNamEMC, c_function_name,
              ethercatmcstrStatus(status), (int)status);
    return status;
  }
  while (counter < MAX_COUNTER) {
    status = getPlcMemoryUint(ctrlLocal.indexerOffset, &value, 2);
    if (status) {
      asynPrint(pasynUserController_,
                ASYN_TRACE_INFO,
                "%s%s:%d readMailboxV3 status=%s (%d)\n",
                modNamEMC, fileName, lineNo,
                ethercatmcstrStatus(status), (int)status);
      return status;
    }
    if (value == valueAcked) {
      status = getPlcMemoryOnErrorStateChange(ctrlLocal.indexerOffset +  1*2,
                                              bufptr, buflen);

      return status;
    }
    counter++;
    epicsThreadSleep(calcSleep(counter));
  }
  status = asynDisabled;
  asynPrint(pasynUserController_,
            ASYN_TRACE_INFO,
            "%s%s descID=0x%X counter=%u value=0x%X status=%s (%d)\n",
            modNamEMC, c_function_name, descID, counter, value,
            ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus
ethercatmcController::newIndexerAxisV3(ethercatmcIndexerAxis *pAxis,
                                       unsigned target_param_descriptor_id,
                                       unsigned auxbits_bitfield_flag_descriptor_id,
                                       unsigned parameters_descriptor_id)
{
  unsigned axisNo = pAxis->axisNo_;
  asynStatus status = asynError;
  {
    allDescriptors_type tmp2Descriptor;
    status = readMailboxV3(target_param_descriptor_id, &tmp2Descriptor, sizeof(tmp2Descriptor));
    if (status) return status;
    double fAbsMin = NETTODOUBLE(tmp2Descriptor.parameterDescriptor.min_value);
    double fAbsMax = NETTODOUBLE(tmp2Descriptor.parameterDescriptor.max_value);
    unsigned unitCode = NETTOUINT(tmp2Descriptor.parameterDescriptor.unit);
    char unitCodeTxt[40];
    unitCodeToText(unitCodeTxt, sizeof(unitCodeTxt), unitCode, "?", "?");

    setStringParam(axisNo,  ethercatmcCfgEGU_RB_, unitCodeTxt);
    setParamMeta(axisNo, ethercatmcCfgPMAX_RB_, "EGU", unitCodeTxt);
    setParamMeta(axisNo, ethercatmcCfgPMIN_RB_, "EGU", unitCodeTxt);

    /* Limits */
    updateCfgValue(axisNo, ethercatmcCfgPMAX_RB_, fAbsMax, "CfgPMAX");
    updateCfgValue(axisNo, ethercatmcCfgPMIN_RB_, fAbsMin, "CfgPMIN");
#ifdef motorHighLimitROString
    udateMotorLimitsRO(axisNo,
                       (fAbsMin > fABSMIN && fAbsMax < fABSMAX),
                       fAbsMax,
                       fAbsMin);
#endif
  }
#ifdef ETHERCATMC_ASYN_PARAMMETA
    pAxis->callParamCallbacks();
#endif

  status = asynSuccess;

  return status;
}

extern "C"  int parameter_is_floatV3(unsigned parameterTypV3)
{
  return !!((parameterTypV3 & 0xC000) == 0x4000);
}

extern "C"  int parameter_is_rw_V3(unsigned parameterTypV3)
{
  return !!((parameterTypV3 & 0x3000) == 0x1000);
}

extern "C" unsigned parameter_has_lenInPlcParaV3(unsigned parameterTypV3)
{
  switch (parameterTypV3 & 0x0C00) {
  case 0x0000: return 0;
  case 0x0400: return 2; // 16 Bit
  case 0x0800: return 4; // 32 Bit
  case 0x0C00: return 8; // 64 Bit
  }
  return 0;
}

extern "C" void parameterTypV3_to_ASCII_V3(char *buf, size_t len,
                                           unsigned parameterTypV3)
{
  const char *unsigned_float_enum_int_str = "hurx";
  const char *function_parameter_str = "hury";
  const char *bit_with_str = "hurz";
  switch (parameterTypV3 & 0xC000) {
    case 0x0000: unsigned_float_enum_int_str = "uint"; break;
    case 0x4000: unsigned_float_enum_int_str = "float"; break;
    case 0x8000: unsigned_float_enum_int_str = "enum"; break;
    case 0xC000: unsigned_float_enum_int_str = "sint"; break;
  }
  switch (parameterTypV3 & 0x3000) {
    case 0x0000: function_parameter_str = "fn"; break;
    case 0x1000: function_parameter_str = "rw"; break;
    case 0x2000: function_parameter_str = "rd"; break;
    case 0x3000: function_parameter_str = "ro"; break;
  }
  switch (parameterTypV3 & 0x0C00) {
    case 0x0000: bit_with_str = "main"; break;
    case 0x0400: bit_with_str = "16"; break;
    case 0x0800: bit_with_str = "32"; break;
    case 0x0C00: bit_with_str = "64"; break;
  }
  snprintf(buf, len, "%s %s %s",
           unsigned_float_enum_int_str,
           function_parameter_str,
           bit_with_str);
}

asynStatus
ethercatmcController::indexerV3readParameterEnums(ethercatmcIndexerAxis *pAxis,
                                                  unsigned parameterIndex,
                                                  unsigned enumparam_read_id,
                                                  unsigned defaultLenInPlcPara)
{
  static const char * const c_function_name = "indexerV3readParameterEnums";
  allDescriptors_type tmp2Descriptor;
  asynStatus status = asynSuccess;

#define MAX_VALUES_FOR_ENUM 16
  /* asyn/asyn/devEpics/devAsynInt32.c */
#define MAX_ENUM_STRING_SIZE 26
  struct {
    char enumChars[MAX_VALUES_FOR_ENUM][MAX_ENUM_STRING_SIZE];
    char *enumStrings[MAX_VALUES_FOR_ENUM];
    int enumValues[MAX_VALUES_FOR_ENUM];
    int enumSeverities[MAX_VALUES_FOR_ENUM];
  } PILSenumsForAsyn;
  unsigned auxBitIdx = 0;
  unsigned enumDescID = enumparam_read_id;
  memset (&PILSenumsForAsyn, 0, sizeof(PILSenumsForAsyn));
  size_t length = sizeof(PILSenumsForAsyn.enumChars[auxBitIdx]) - 1;
  while (!status && enumDescID && (auxBitIdx < MAX_VALUES_FOR_ENUM)) {
    status = readMailboxV3(enumDescID,
                           &tmp2Descriptor, sizeof(tmp2Descriptor));
    if (status) return status;
    unsigned enum_value = NETTOUINT(tmp2Descriptor.enumDescriptor.enum_value);
    enumDescID = NETTOUINT(tmp2Descriptor.enumDescriptor.prev_descriptor_id);
    asynPrint(pasynUserController_, ASYN_TRACE_INFO,
              "%s%s descID=0x%04X type=0x%X enumDescriptor prev=0x%04X enum_value=%u utf8_string=\"%s\"\n",
              modNamEMC, c_function_name, enumDescID,
              NETTOUINT(tmp2Descriptor.enumDescriptor.descriptor_type_0x4006),
              NETTOUINT(tmp2Descriptor.enumDescriptor.prev_descriptor_id),
              enum_value,
              tmp2Descriptor.enumDescriptor.enum_name);
    strncpy(PILSenumsForAsyn.enumChars[auxBitIdx],
            tmp2Descriptor.enumDescriptor.enum_name,
            length);
    PILSenumsForAsyn.enumStrings[auxBitIdx] = &PILSenumsForAsyn.enumChars[auxBitIdx][0];
    PILSenumsForAsyn.enumValues[auxBitIdx] = enum_value;
    auxBitIdx++;
  }
  if (parameterIndex == PARAM_IDX_HOMPROC_FLOAT) {
    for (unsigned i = 0; i < auxBitIdx; i++) {
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s(%i) [%u] enumString=\"%s\" enumValue=%i\n",
                modNamEMC, c_function_name, pAxis->axisNo_, i,
                PILSenumsForAsyn.enumStrings[i],
                PILSenumsForAsyn.enumValues[i]);
#ifdef ETHERCATMC_ASYN_PARAMMETA
      char name_str[4];
      epicsSnprintf(name_str, sizeof(name_str), "%u", i);
      setParamMeta(pAxis->axisNo_, ethercatmcHomProc_RB_, name_str,
                   PILSenumsForAsyn.enumStrings[i]);
      setParamMeta(pAxis->axisNo_, ethercatmcHomProc_RB_, name_str,
                   PILSenumsForAsyn.enumValues[i]);
#endif
    }
#ifdef ETHERCATMC_ASYN_PARAMMETA
    pAxis->callParamCallbacks();
#else
    doCallbacksEnum(PILSenumsForAsyn.enumStrings,
                    PILSenumsForAsyn.enumValues,
                    PILSenumsForAsyn.enumSeverities,
                    auxBitIdx,
                    ethercatmcHomProc_RB_,  pAxis->axisNo_);
#endif
  }
  return status;
}

asynStatus
ethercatmcController::indexerV3readParameterDescriptors(ethercatmcIndexerAxis *pAxis,
                                                        unsigned descID,
                                                        unsigned target_param_descriptor_id,
                                                        unsigned defaultLenInPlcPara)
{
  static const char * const c_function_name = "indexerV3readParameterDescriptors";
  asynStatus status = asynSuccess;
  allDescriptors_type tmp2Descriptor;
  status = readMailboxV3(target_param_descriptor_id, &tmp2Descriptor, sizeof(tmp2Descriptor));
  if (status) return status;
  unsigned unitCode = NETTOUINT(tmp2Descriptor.parameterDescriptor.unit);

  const char *mainBaseUnitTxt = getPlcBaseUnitTxtFromUnitCodeV3(unitCode);
  const char *mainTimeBaseTxt = getPlcTimeBaseTxtFromUnitCodeV3(unitCode);
  const char *mainUnitExponentTxt = getPlcUnitExponentTxtV3(unitCode);
  if (!mainBaseUnitTxt) mainBaseUnitTxt = "?";
  if (!mainTimeBaseTxt) mainTimeBaseTxt = "??";
  if (!mainUnitExponentTxt) mainUnitExponentTxt = "????";

  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s descID=0x%04X target_param_descriptor_id=%u unit=\"%s%s%s\"\n",
            modNamEMC, c_function_name, descID, target_param_descriptor_id,
            mainBaseUnitTxt,
            mainTimeBaseTxt,
            mainUnitExponentTxt);

  while (!status && descID) {
    allDescriptors_type tmp1Descriptor;
    status = readMailboxV3(descID,
                           &tmp1Descriptor, sizeof(tmp1Descriptor));
    unsigned descriptor_type_XXXX = NETTOUINT(tmp1Descriptor.genericDescriptor.descriptor_type);
    unsigned parameterIndex = 0;
    unsigned parameterTypV3 = 0;
    unsigned enumparam_read_id = 0;
    unsigned enumparam_write_id = 0;
    double   max_value = 0.0;
    char unitCodeTxt[40];
    unitCodeTxt[0] = '\0';

    NETTOUINT(tmp1Descriptor.deviceDescriptor.prev_descriptor_id);
    switch (descriptor_type_XXXX) {
    case 0x6114:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmp1Descriptor.parameterDescriptor.prev_descriptor_id);
        parameterIndex = NETTOUINT(tmp1Descriptor.parameterDescriptor.parameterIndex);
        parameterTypV3 = NETTOUINT(tmp1Descriptor.parameterDescriptor.parameterTypV3);
        max_value = NETTODOUBLE(tmp1Descriptor.parameterDescriptor.max_value);
        char parameterTypV3_ascii[32];
        unitCode = NETTOUINT(tmp1Descriptor.parameterDescriptor.unit);
        unitCodeToText(unitCodeTxt, sizeof(unitCodeTxt), unitCode,
                       mainBaseUnitTxt, mainUnitExponentTxt);

        parameterTypV3_to_ASCII_V3(parameterTypV3_ascii,
                                   sizeof(parameterTypV3_ascii),
                                   parameterTypV3);

        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameterIndex=%u type=0x%X parameterDescriptor"
                  " prev=0x%04X string=0x%04X param_type=0x%04X (%s) unit=\"%s\" (0x%x) min=%f max=%f utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameterIndex,
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.descriptor_type_0x6114),
                  prev_descriptor_id,
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.string_description_id),
                  parameterTypV3, parameterTypV3_ascii,
                  unitCodeTxt, unitCode,
                  NETTODOUBLE(tmp1Descriptor.parameterDescriptor.min_value),
                  NETTODOUBLE(tmp1Descriptor.parameterDescriptor.max_value),
                  tmp1Descriptor.parameterDescriptor.parameter_name);

        descID = prev_descriptor_id;
      }
      break;
    case 0x620E:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmp1Descriptor.enumparamDescriptor.prev_descriptor_id);
        parameterIndex = NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_index);
        parameterTypV3 = NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_type);
        enumparam_read_id = NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_read_id);
        enumparam_write_id = NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_write_id);
        char parameterTypV3_ascii[32];
        parameterTypV3_to_ASCII_V3(parameterTypV3_ascii,
                                   sizeof(parameterTypV3_ascii),
                                   parameterTypV3);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameterIndex=%u type=0x%X enumparamDescriptor"
                  " prev=0x%04X string=0x%04X  read_id=0x%04X write_id=0x04%x param_type=0x%X (%s) utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameterIndex,
                  NETTOUINT(tmp1Descriptor.enumparamDescriptor.descriptor_type_0x620e),
                  NETTOUINT(tmp1Descriptor.enumparamDescriptor.prev_descriptor_id),
                  NETTOUINT(tmp1Descriptor.enumparamDescriptor.string_description_id),
                  enumparam_read_id,
                  enumparam_write_id,
                  parameterTypV3, parameterTypV3_ascii,
                  tmp1Descriptor.enumparamDescriptor.enumparam_name);
        descID = prev_descriptor_id;
      }
      break;
    case 0x680E:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmp1Descriptor.functionDescriptor.prev_descriptor_id);
        unsigned function_argument_id = NETTOUINT(tmp1Descriptor.functionDescriptor.function_argument_id);
        unsigned function_result_id = NETTOUINT(tmp1Descriptor.functionDescriptor.function_result_id);
        parameterIndex = NETTOUINT(tmp1Descriptor.functionDescriptor.function_index);
        if (function_argument_id) {
          allDescriptors_type tmp3Descriptor;
          status = readMailboxV3(function_argument_id, &tmp3Descriptor, sizeof(tmp3Descriptor));
          if (status) return status;
          /* Take the parameter type from the paramater descriptor */
          parameterTypV3 = NETTOUINT(tmp3Descriptor.parameterDescriptor.parameterTypV3);
        } else {
          parameterTypV3 = 0x1000; /* A function is writable */
        }
        char parameterTypV3_ascii[32];
        parameterTypV3_to_ASCII_V3(parameterTypV3_ascii,
                                   sizeof(parameterTypV3_ascii),
                                   parameterTypV3);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameterIndex=%u type=0x%X functionDescriptor"
                  " prev=0x%04X string=0x%04X  arg_id=0x%x res_id=0x%x param_type=0x%X (%s) fun_idx=%d flags=%x utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameterIndex,
                  NETTOUINT(tmp1Descriptor.functionDescriptor.descriptor_type_0x680e),
                  prev_descriptor_id,
                  NETTOUINT(tmp1Descriptor.functionDescriptor.string_description_id),
                  function_argument_id, function_result_id,
                  parameterTypV3, parameterTypV3_ascii,
                  NETTOUINT(tmp1Descriptor.functionDescriptor.function_index),
                  NETTOUINT(tmp1Descriptor.functionDescriptor.function_flags),
                  tmp1Descriptor.functionDescriptor.function_name);
        descID = prev_descriptor_id;
      }
      break;
    default:
      {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X unsupported\n",
                  modNamEMC, c_function_name,
                  descID, descriptor_type_XXXX);
        descID = 0;
      }
      break;
    }
    if (parameterIndex) {
      int index_in_range;
      index_in_range = parameterIndex < (sizeof(pAxis->drvlocal.PILSparamPerm) /
                                          sizeof(pAxis->drvlocal.PILSparamPerm[0]));
      if (index_in_range) {
        int parameter_is_float = parameter_is_floatV3(parameterTypV3);
        int parameter_is_rw = parameter_is_rw_V3(parameterTypV3);
        unsigned lenInPlcPara = parameter_has_lenInPlcParaV3(parameterTypV3);
        if (!lenInPlcPara) {
          lenInPlcPara = defaultLenInPlcPara;
        }
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s parameterIndex=%u parameter_is_float=%d parameter_is_rw=%d unit=\"%s\" lenInPlcPara=%u\n",
                  modNamEMC, c_function_name, parameterIndex,
                  parameter_is_float, parameter_is_rw, unitCodeTxt, lenInPlcPara);
        if (parameterIndex == PARAM_IDX_OPMODE_AUTO_UINT) {
          /* Special case for EPICS: We do not poll it in background */
          pAxis->setIntegerParam(motorStatusGainSupport_, 1);
        }
        pAxis->drvlocal.PILSparamPerm[parameterIndex] =
          parameter_is_rw ? PILSparamPermWrite : PILSparamPermRead;
        if (parameter_is_float) {
          pAxis->drvlocal.lenInPlcParaFloat[parameterIndex] = lenInPlcPara;
        } else {
          pAxis->drvlocal.lenInPlcParaInteger[parameterIndex] = lenInPlcPara;
        }
        if (enumparam_read_id && (enumparam_read_id == enumparam_write_id)) {
          pAxis->drvlocal.enumparam_read_id[parameterIndex] = enumparam_read_id;
        }
        {
          /* Max velocity for VMAX */
          if (parameterIndex == PARAM_IDX_SPEED_FLOAT) {
            int initial = 1;
            parameterFloatReadBack(pAxis->axisNo_,
                                   initial,
                                   PARAM_IDX_MAX_VELO_FLOAT,
                                   max_value);
          }
            /* Set EGU. asyn that supports setParamMeta() is needed to make this work */
          if (parameterIndex == PARAM_IDX_HYTERESIS_FLOAT) {
            setParamMeta(pAxis->axisNo_, ethercatmcCfgSPDB_RB_, "EGU", unitCodeTxt);
            setParamMeta(pAxis->axisNo_, ethercatmcCfgRDBD_RB_, "EGU", unitCodeTxt);
          } else if (unitCodeTxt[0]) {
            int function = paramIndexToFunction(parameterIndex);
            if (function) {
              setParamMeta(pAxis->axisNo_, ethercatmcCfgSPDB_RB_, "EGU", unitCodeTxt);
            }
          }
        }
      }
    }
  }
  return status;
}

asynStatus
ethercatmcController::indexerV3readAuxbits(ethercatmcIndexerAxis *pAxis,
                                           unsigned descID)
{
  static const char * const c_function_name = "indexerV3readAuxbits";
  unsigned axisNo = pAxis->axisNo_;
  asynStatus status = asynSuccess;

  while (!status && descID) {
    allDescriptors_type tmp1Descriptor;
    unsigned prev_descriptor_id;
    status = readMailboxV3(descID,
                           &tmp1Descriptor, sizeof(tmp1Descriptor));
    unsigned descriptor_type_XXXX = NETTOUINT(tmp1Descriptor.genericDescriptor.descriptor_type);
    prev_descriptor_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.prev_descriptor_id);
    switch (descriptor_type_XXXX) {
    case 0x5008:
      {
        prev_descriptor_id = NETTOUINT(tmp1Descriptor.bitfieldDescriptor.prev_descriptor_id);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X bitfieldDescriptor prev=0x%04X last=0x%04X lowest=%u width=%u utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  NETTOUINT(tmp1Descriptor.bitfieldDescriptor.descriptor_type_0x5008),
                  NETTOUINT(tmp1Descriptor.bitfieldDescriptor.prev_descriptor_id),
                  NETTOUINT(tmp1Descriptor.bitfieldDescriptor.last_descriptor_id),
                  NETTOUINT(tmp1Descriptor.bitfieldDescriptor.lowest_bit),
                  NETTOUINT(tmp1Descriptor.bitfieldDescriptor.bit_width),
                  tmp1Descriptor.bitfieldDescriptor.bitfield_name);
        descID = prev_descriptor_id;
        // break the loop
        //descID = 0; // NETTOUINT(tmp1Descriptor.bitfieldDescriptor.last_descriptor_id);
      }
      break;
    case 0x5105:
      {
        prev_descriptor_id = NETTOUINT(tmp1Descriptor.flagDescriptor.prev_descriptor_id);
        unsigned bit_number = NETTOUINT(tmp1Descriptor.flagDescriptor.bit_number);
        const char *flag_name = &tmp1Descriptor.flagDescriptor.flag_name[0];
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X flagDescriptor prev=0x%04X bit_number=%u utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  descriptor_type_XXXX,
                  prev_descriptor_id,
                  bit_number,
                  flag_name);
        int function = ethercatmcNamAux0_ + bit_number;
        if (function <= ethercatmcNamAux0_ + MAX_AUX_BIT_SHOWN) {
          pAxis->setStringParam(function, flag_name);
          setAlarmStatusSeverityWrapper(axisNo, function, asynSuccess);
        }
        if (!strcmp("notHomed", flag_name)) {
          pAxis->setAuxBitsNotHomedMask(1 << bit_number);
        } else if (!strcmp("enabled", flag_name)) {
          pAxis->setAuxBitsEnabledMask(1 << bit_number);
        } else if (!strcmp("localMode", flag_name)) {
          pAxis->setAuxBitsLocalModeMask(1 << bit_number);
        } else if (!strcmp("homeSwitch", flag_name)) {
          pAxis->setAuxBitsHomeSwitchMask(1 << bit_number);
        }
        descID = prev_descriptor_id;
      }
      break;
    default:
      {
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X\n",
                  modNamEMC, c_function_name,
                  descID, descriptor_type_XXXX);
        descID = 0;
      }
      break;
    }
  }
  return status;
}

asynStatus
ethercatmcController::indexerV3addDevice(unsigned devNum,
                                         int axisNo,
                                         unsigned iOffsBytes,
                                         unsigned string_description_id,
                                         unsigned target_param_descriptor_id,
                                         unsigned auxbits_bitfield_flag_descriptor_id,
                                         unsigned parameters_descriptor_id,
                                         unsigned enum_errorId_descriptor_id,
                                         unsigned type_code,
                                         unsigned device_offset,
                                         unsigned device_flags,
                                         const char *device_name)
{
  static const char * const c_function_name = "indexerV3addDevice";
  asynStatus status = asynError;
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s(%d) iOffsBytes=%u string=0x%04X target=0x%04X auxbits=0x%04X paramters=0x%04X enum_errorID=0x%04X type_code=0x%04X offset=%u flags=0x%x name=\"%s\"\n",
            modNamEMC, c_function_name, axisNo,
            iOffsBytes,
            string_description_id,
            target_param_descriptor_id,
            auxbits_bitfield_flag_descriptor_id,
            parameters_descriptor_id,
            enum_errorId_descriptor_id,
            type_code,
            device_offset,
            device_flags,
            device_name);
    switch (type_code) {
    case 0x1E04:
    case 0x5008:
    case 0x500C:
    case 0x5010:
      {
        ethercatmcIndexerAxis *pAxis;
        pAxis = static_cast<ethercatmcIndexerAxis*>(asynMotorController::getAxis(axisNo));
        if (!pAxis) {
          pAxis = new ethercatmcIndexerAxis(this, axisNo, 0, NULL);
        }
        /* Now we have an axis */
        pAxis->setIndexerDevNumOffsetTypeCode(devNum, device_offset, type_code);
        setStringParam(axisNo,  ethercatmcCfgDESC_RB_, device_name);
        status = newIndexerAxisV3(pAxis,
                                  target_param_descriptor_id,
                                  auxbits_bitfield_flag_descriptor_id,
                                  parameters_descriptor_id);
        if (!status) {
          unsigned descID = auxbits_bitfield_flag_descriptor_id;
          status = indexerV3readAuxbits(pAxis, descID);
        }
        if (!status) {
          if (type_code == 0x5010) {
            unsigned descID = parameters_descriptor_id;
            unsigned defaultLenInPlcPara = 8;
            status = indexerV3readParameterDescriptors(pAxis, descID,
                                                       target_param_descriptor_id,
                                                       defaultLenInPlcPara);
          }
        }
      }
    }
    return status;
}


asynStatus ethercatmcController::indexerInitialPollv3(void)
{
  static const char * const c_function_name = "indexerInitialPollv3";
  int      axisNo = 0;
  unsigned devNum = 0;
  unsigned numOfDevicesInPLC = 100;
  unsigned firstDeviceStartOffset = (unsigned)-1; /* Will be decreased while we go */
  unsigned lastDeviceEndOffset = 0;  /* will be increased while we go */
  asynStatus status;
  allDescriptors_type tmp1Descriptor;
  for (unsigned descID = 0; descID < 100; descID++) {
    status = readMailboxV3(descID, &tmp1Descriptor, sizeof(tmp1Descriptor));
    if (status) goto endPollIndexer3;
    unsigned descriptor_type_XXXX = NETTOUINT(tmp1Descriptor.genericDescriptor.descriptor_type);
    unsigned descriptor_prev_XXXX = 0x7FFF & NETTOUINT(tmp1Descriptor.genericDescriptor.descriptor_prev);
    if (status) goto endPollIndexer3;
    switch (descriptor_type_XXXX) {
    case 0x1010:
      numOfDevicesInPLC = NETTOUINT(tmp1Descriptor.plcDescriptor.number_of_devices);
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%04X  plcDescriptor last=0x%04X plcdesc=0x%04X plcvers=0x%04X plcauthor=0x%04X slot_size=%u num_of_devices=%u flags=0x%x name=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.plcDescriptor.descriptor_type_0x1010),
                NETTOUINT(tmp1Descriptor.plcDescriptor.last_descriptor_id),
                NETTOUINT(tmp1Descriptor.plcDescriptor.plc_description_id),
                NETTOUINT(tmp1Descriptor.plcDescriptor.plc_version_descriptor_id),
                NETTOUINT(tmp1Descriptor.plcDescriptor.plc_author_descriptor_id),
                NETTOUINT(tmp1Descriptor.plcDescriptor.descriptor_slot_size),
                numOfDevicesInPLC,
                NETTOUINT(tmp1Descriptor.plcDescriptor.plc_flags),
                tmp1Descriptor.plcDescriptor.name_of_plc);
#ifdef motorMessageTextString
      (void)setStringParam(motorMessageText_, tmp1Descriptor.plcDescriptor.name_of_plc);
#endif
      break;

    case 0x2014:
      {
        unsigned string_description_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.string_description_id);
        unsigned target_param_descriptor_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.target_param_descriptor_id);
        unsigned auxbits_bitfield_flag_descriptor_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.auxbits_bitfield_flag_descriptor_id);
        unsigned parameters_descriptor_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.parameters_descriptor_id);
        unsigned enum_errorId_descriptor_id = NETTOUINT(tmp1Descriptor.deviceDescriptor.enum_errorId_descriptor_id);
        unsigned type_code = NETTOUINT(tmp1Descriptor.deviceDescriptor.type_code);
        unsigned device_offset = NETTOUINT(tmp1Descriptor.deviceDescriptor.device_offset);
        unsigned device_flags = NETTOUINT(tmp1Descriptor.deviceDescriptor.device_flags);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X deviceDescriptor prev=0x%04X string=0x%04X target=0x%04X auxbits=0x%04X paramters=0x%04X enum_errorID=0x%04X type_code=0x%04X offset=%u flags=0x%x name=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  NETTOUINT(tmp1Descriptor.deviceDescriptor.descriptor_type_0x2014),
                  NETTOUINT(tmp1Descriptor.deviceDescriptor.prev_descriptor_id),
                  string_description_id,
                  target_param_descriptor_id,
                  auxbits_bitfield_flag_descriptor_id,
                  parameters_descriptor_id,
                  enum_errorId_descriptor_id,
                  type_code,
                  device_offset,
                  device_flags,
                  tmp1Descriptor.deviceDescriptor.device_name);
        switch (type_code) {
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
        devNum++;
        status = indexerV3addDevice(devNum, axisNo,
                                    device_offset,
                                    string_description_id,
                                    target_param_descriptor_id,
                                    auxbits_bitfield_flag_descriptor_id,
                                    parameters_descriptor_id,
                                    enum_errorId_descriptor_id,
                                    type_code,
                                    device_offset,
                                    device_flags,
                                    tmp1Descriptor.deviceDescriptor.device_name);
        if (status) goto endPollIndexer3;
        unsigned iSizeBytes = (type_code & 0XFF) * 2;
        unsigned endOffset = device_offset + iSizeBytes;

        /* find the lowest and highest offset for all devices */
        if (device_offset < firstDeviceStartOffset) {
          firstDeviceStartOffset = device_offset;
        }
        if (endOffset > lastDeviceEndOffset) {
          lastDeviceEndOffset = endOffset;
        }
      }
      break;
    case 0x3004:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X stringDescriptor prev=0x%04X utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.stringDescriptor.descriptor_type_0x3004),
                NETTOUINT(tmp1Descriptor.stringDescriptor.prev_descriptor_id),
                tmp1Descriptor.stringDescriptor.utf8_string);
      break;

    case 0x4006:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X enumDescriptor prev=0x%04X enum_value=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.enumDescriptor.descriptor_type_0x4006),
                NETTOUINT(tmp1Descriptor.enumDescriptor.prev_descriptor_id),
                NETTOUINT(tmp1Descriptor.enumDescriptor.enum_value),
                tmp1Descriptor.enumDescriptor.enum_name);
      break;
    case 0x5008:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X bitfieldDescriptor prev=0x%04X last=0x%04X lowest=%u width=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.bitfieldDescriptor.descriptor_type_0x5008),
                NETTOUINT(tmp1Descriptor.bitfieldDescriptor.prev_descriptor_id),
                NETTOUINT(tmp1Descriptor.bitfieldDescriptor.last_descriptor_id),
                NETTOUINT(tmp1Descriptor.bitfieldDescriptor.lowest_bit),
                NETTOUINT(tmp1Descriptor.bitfieldDescriptor.bit_width),
                tmp1Descriptor.bitfieldDescriptor.bitfield_name);
      break;
    case 0x5105:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X flagDescriptor prev=0x%04X bit_number=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.flagDescriptor.descriptor_type_0x5105),
                NETTOUINT(tmp1Descriptor.flagDescriptor.prev_descriptor_id),
                NETTOUINT(tmp1Descriptor.flagDescriptor.bit_number),
                tmp1Descriptor.flagDescriptor.flag_name);
      break;
    case 0x6114:
      {
        unsigned unitCode = NETTOUINT(tmp1Descriptor.parameterDescriptor.unit);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X parameterDescriptor prev=0x%04X string=0x%04X index=%u type=0x%04X unit=0x%x min=%f max=%f utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.descriptor_type_0x6114),
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.prev_descriptor_id),
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.string_description_id),
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.parameterIndex),
                  NETTOUINT(tmp1Descriptor.parameterDescriptor.parameterTypV3),
                  unitCode,
                  NETTODOUBLE(tmp1Descriptor.parameterDescriptor.min_value),
                  NETTODOUBLE(tmp1Descriptor.parameterDescriptor.max_value),
                  tmp1Descriptor.parameterDescriptor.parameter_name);
      }
      break;
    case 0x620e:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X enumparamDescriptor prev=0x%04X string=0x%04X read=0x%04X write=0x%04X index=0x%04X type=0x%04X utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.descriptor_type_0x620e),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.prev_descriptor_id),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.string_description_id),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_read_id),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_write_id),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_index),
                NETTOUINT(tmp1Descriptor.enumparamDescriptor.enumparam_type),
                tmp1Descriptor.enumparamDescriptor.enumparam_name);
      break;
    case 0x680c:
    case 0x680e:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s functionDescriptor descID=0x%04X type=0x%X prev=0x%04X string=0x%04X argument=0x%04X result=0x%04X index=0x%04X flags=0x%x utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.functionDescriptor.descriptor_type_0x680e),
                NETTOUINT(tmp1Descriptor.functionDescriptor.prev_descriptor_id),
                NETTOUINT(tmp1Descriptor.functionDescriptor.string_description_id),
                NETTOUINT(tmp1Descriptor.functionDescriptor.function_argument_id),
                NETTOUINT(tmp1Descriptor.functionDescriptor.function_result_id),
                NETTOUINT(tmp1Descriptor.functionDescriptor.function_index),
                NETTOUINT(tmp1Descriptor.functionDescriptor.function_flags),
                tmp1Descriptor.functionDescriptor.function_name);
      break;
    case 0x7FFF:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X debugDescriptor cycleCounter=0x%04X lengthOfDebugDescriptor=%u name=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmp1Descriptor.debugDescriptor.descriptor_type_0xFFFF),
                NETTOUINT(tmp1Descriptor.debugDescriptor.cycleCounter),
                NETTOUINT(tmp1Descriptor.debugDescriptor.lengthOfDebugDescriptor),
                tmp1Descriptor.debugDescriptor.message);
      break;
    default:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X XXXXXdescriptor prev=0x%04X\n",
                modNamEMC, c_function_name, descID,
                descriptor_type_XXXX,
                descriptor_prev_XXXX);
    }
#if 0
    if (devNum >= numOfDevicesInPLC)
      break;
#else
    if (!descriptor_type_XXXX)
      break;
#endif
  }
  status = asynSuccess;
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
 endPollIndexer3:
  asynPrint(pasynUserController_, ASYN_TRACE_INFO,
            "%s%s status=%d\n",
            modNamEMC, c_function_name, status);
  return status;
}

