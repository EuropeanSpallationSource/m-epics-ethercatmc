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
    uint8_t parameter_index                          [2];
    uint8_t parameter_type                           [2];
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
  asynStatus status;
  unsigned value = descID;
  unsigned valueAcked = 0x8000 + value;
  unsigned counter = 0;
  if (descID > 0x7FFF) {
    status = asynDisabled;
    asynPrint(pasynUserController_,
              ASYN_TRACE_INFO,
              "%s%s:%d readDeviceIndexer descID=%d status=%s (%d)\n",
              modNamEMC, fileName, lineNo, descID,
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
              "%s%s:%d readMailboxV3 status=%s (%d)\n",
              modNamEMC,fileName, lineNo,
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
            "%sreadDeviceIndexer descID=0x%X counter=%u value=0x%X status=%s (%d)\n",
            modNamEMC, descID, counter, value,
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
    allDescriptors_type tmpDescriptor;
    status = readMailboxV3(target_param_descriptor_id, &tmpDescriptor, sizeof(tmpDescriptor));
    if (status) return status;
    double fAbsMin = NETTODOUBLE(tmpDescriptor.parameterDescriptor.min_value);
    double fAbsMax = NETTODOUBLE(tmpDescriptor.parameterDescriptor.max_value);
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

  status = asynSuccess;

  return status;
}

extern int parameter_is_floatV3(unsigned parameter_type)
{
  return !!((parameter_type & 0xC000) == 0x4000);
}

extern int parameter_is_rw_V3(unsigned parameter_type)
{
  return !!((parameter_type & 0x3000) == 0x1000);
}

extern unsigned parameter_has_lenInPlcParaV3(unsigned parameter_type)
{
  switch (parameter_type & 0x0C00) {
  case 0x0000: return 0;
  case 0x0400: return 2; // 16 Bit
  case 0x0800: return 4; // 32 Bit
  case 0x0C00: return 8; // 64 Bit
  }
  return 0;
}

extern "C" void parameter_type_to_ASCII_V3(char *buf, size_t len,
                                           unsigned parameter_type)
{
  const char *unsigned_float_enum_int_str = "hurx";
  const char *function_parameter_str = "hury";
  const char *bit_with_str = "hurz";
  switch (parameter_type & 0xC000) {
    case 0x0000: unsigned_float_enum_int_str = "uint"; break;
    case 0x4000: unsigned_float_enum_int_str = "float"; break;
    case 0x8000: unsigned_float_enum_int_str = "enum"; break;
    case 0xC000: unsigned_float_enum_int_str = "sint"; break;
  }
  switch (parameter_type & 0x3000) {
    case 0x0000: function_parameter_str = "fn"; break;
    case 0x1000: function_parameter_str = "rw"; break;
    case 0x2000: function_parameter_str = "rd"; break;
    case 0x3000: function_parameter_str = "rf"; break;
  }
  switch (parameter_type & 0x0C00) {
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
ethercatmcController::indexerV3readParameterDescriptors(ethercatmcIndexerAxis *pAxis,
                                                        unsigned descID,
                                                        unsigned defaultLenInPlcPara)
{
  static const char * const c_function_name = "indexerV3readParameterDescriptors";
  asynStatus status = asynSuccess;

  while (!status && descID) {
    allDescriptors_type tmpDescriptor;
    status = readMailboxV3(descID,
                           &tmpDescriptor, sizeof(tmpDescriptor));
    unsigned descriptor_type_XXXX = NETTOUINT(tmpDescriptor.genericDescriptor.descriptor_type);
    unsigned parameter_index = 0;
    unsigned parameter_type = 0;
    NETTOUINT(tmpDescriptor.deviceDescriptor.prev_descriptor_id);
    switch (descriptor_type_XXXX) {
    case 0x6114:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmpDescriptor.parameterDescriptor.prev_descriptor_id);
        parameter_index = NETTOUINT(tmpDescriptor.parameterDescriptor.parameter_index);
        parameter_type = NETTOUINT(tmpDescriptor.parameterDescriptor.parameter_type);
        char parameter_type_ascii[32];
        parameter_type_to_ASCII_V3(parameter_type_ascii,
                                   sizeof(parameter_type_ascii),
                                   parameter_type);

        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameter_index=%u type=0x%X parameterDescriptor"
                  " prev=0x%04X string=0x%04X param_type=0x%04X (%s) unit=0x%x min=%f max=%f utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameter_index,
                  NETTOUINT(tmpDescriptor.parameterDescriptor.descriptor_type_0x6114),
                  prev_descriptor_id,
                  NETTOUINT(tmpDescriptor.parameterDescriptor.string_description_id),
                  parameter_type, parameter_type_ascii,
                  NETTOUINT(tmpDescriptor.parameterDescriptor.unit),
                  NETTODOUBLE(tmpDescriptor.parameterDescriptor.min_value),
                  NETTODOUBLE(tmpDescriptor.parameterDescriptor.max_value),
                  tmpDescriptor.parameterDescriptor.parameter_name);

        descID = prev_descriptor_id;
      }
      break;
    case 0x620E:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmpDescriptor.enumparamDescriptor.prev_descriptor_id);
        parameter_index = NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_index);
        parameter_type = NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_type);
        char parameter_type_ascii[32];
        parameter_type_to_ASCII_V3(parameter_type_ascii,
                                   sizeof(parameter_type_ascii),
                                   parameter_type);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameter_index=%u type=0x%X enumparamDescriptor"
                  " prev=0x%04X string=0x%04X  read_id=0x%x write_id=0x%x param_type=0x%X (%s) utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameter_index,
                  NETTOUINT(tmpDescriptor.enumparamDescriptor.descriptor_type_0x620e),
                  NETTOUINT(tmpDescriptor.enumparamDescriptor.prev_descriptor_id),
                  NETTOUINT(tmpDescriptor.enumparamDescriptor.string_description_id),
                  NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_read_id),
                  NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_write_id),
                  parameter_type, parameter_type_ascii,
                  tmpDescriptor.enumparamDescriptor.enumparam_name);
        descID = prev_descriptor_id;
      }
      break;
    case 0x680E:
      {
        unsigned prev_descriptor_id = NETTOUINT(tmpDescriptor.functionDescriptor.prev_descriptor_id);
        parameter_index = NETTOUINT(tmpDescriptor.functionDescriptor.function_index);
        parameter_type = 0x1000; /* A function is writable */
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X parameter_index=%u type=0x%X functionDescriptor"
                  " prev=0x%04X string=0x%04X  arg_id=0x%x res_id=0x%x fun_idx=%d flags=%x utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID, parameter_index,
                  NETTOUINT(tmpDescriptor.functionDescriptor.descriptor_type_0x680e),
                  prev_descriptor_id,
                  NETTOUINT(tmpDescriptor.functionDescriptor.string_description_id),
                  NETTOUINT(tmpDescriptor.functionDescriptor.function_argument_id),
                  NETTOUINT(tmpDescriptor.functionDescriptor.function_result_id),
                  NETTOUINT(tmpDescriptor.functionDescriptor.function_index),
                  NETTOUINT(tmpDescriptor.functionDescriptor.function_flags),
                  tmpDescriptor.functionDescriptor.function_name);
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
    if (parameter_index) {
      int index_in_range;
      index_in_range = parameter_index < (sizeof(pAxis->drvlocal.PILSparamPerm) /
                                          sizeof(pAxis->drvlocal.PILSparamPerm[0]));
      if (index_in_range) {
        int parameter_is_float = parameter_is_floatV3(parameter_type);
        int parameter_is_rw = parameter_is_rw_V3(parameter_type);
        unsigned lenInPlcPara = parameter_has_lenInPlcParaV3(parameter_type);
        if (!lenInPlcPara) {
          lenInPlcPara = defaultLenInPlcPara;
        }
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s parameter_index=%u parameter_is_float=%d parameter_is_rw=%d lenInPlcPara=%u\n",
                  modNamEMC, c_function_name, parameter_index,
                  parameter_is_float, parameter_is_rw, lenInPlcPara);
        if (parameter_index == PARAM_IDX_OPMODE_AUTO_UINT) {
          /* Special case for EPICS: We d not poll it in background */
          pAxis->setIntegerParam(motorStatusGainSupport_, 1);
        }
        pAxis->drvlocal.PILSparamPerm[parameter_index] =
          parameter_is_rw ? PILSparamPermWrite : PILSparamPermRead;
        if (parameter_is_float) {
          pAxis->drvlocal.lenInPlcParaFloat[parameter_index] = lenInPlcPara;
        } else {
          pAxis->drvlocal.lenInPlcParaInteger[parameter_index] = lenInPlcPara;
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
    allDescriptors_type tmpDescriptor;
    unsigned prev_descriptor_id;
    status = readMailboxV3(descID,
                           &tmpDescriptor, sizeof(tmpDescriptor));
    unsigned descriptor_type_XXXX = NETTOUINT(tmpDescriptor.genericDescriptor.descriptor_type);
    prev_descriptor_id = NETTOUINT(tmpDescriptor.deviceDescriptor.prev_descriptor_id);
    switch (descriptor_type_XXXX) {
    case 0x5008:
      {
        prev_descriptor_id = NETTOUINT(tmpDescriptor.bitfieldDescriptor.prev_descriptor_id);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X bitfieldDescriptor prev=0x%04X last=0x%04X lowest=%u width=%u utf8_string=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  NETTOUINT(tmpDescriptor.bitfieldDescriptor.descriptor_type_0x5008),
                  NETTOUINT(tmpDescriptor.bitfieldDescriptor.prev_descriptor_id),
                  NETTOUINT(tmpDescriptor.bitfieldDescriptor.last_descriptor_id),
                  NETTOUINT(tmpDescriptor.bitfieldDescriptor.lowest_bit),
                  NETTOUINT(tmpDescriptor.bitfieldDescriptor.bit_width),
                  tmpDescriptor.bitfieldDescriptor.bitfield_name);
        descID = prev_descriptor_id;
        // break the loop
        //descID = 0; // NETTOUINT(tmpDescriptor.bitfieldDescriptor.last_descriptor_id);
      }
      break;
    case 0x5105:
      {
        prev_descriptor_id = NETTOUINT(tmpDescriptor.flagDescriptor.prev_descriptor_id);
        unsigned bit_number = NETTOUINT(tmpDescriptor.flagDescriptor.bit_number);
        const char *flag_name = &tmpDescriptor.flagDescriptor.flag_name[0];
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
        char unitCodeTxt[40];
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
            status = indexerV3readParameterDescriptors(pAxis, descID, defaultLenInPlcPara);
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
  allDescriptors_type tmpDescriptor;
  for (unsigned descID = 0; descID < 100; descID++) {
    status = readMailboxV3(descID, &tmpDescriptor, sizeof(tmpDescriptor));
    if (status) goto endPollIndexer3;
    unsigned descriptor_type_XXXX = NETTOUINT(tmpDescriptor.genericDescriptor.descriptor_type);
    unsigned descriptor_prev_XXXX = 0x7FFF & NETTOUINT(tmpDescriptor.genericDescriptor.descriptor_prev);
    if (status) goto endPollIndexer3;
    switch (descriptor_type_XXXX) {
    case 0x1010:
      numOfDevicesInPLC = NETTOUINT(tmpDescriptor.plcDescriptor.number_of_devices);
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%04X  plcDescriptor last=0x%04X plcdesc=0x%04X plcvers=0x%04X plcauthor=0x%04X slot_size=%u num_of_devices=%u flags=0x%x name=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.plcDescriptor.descriptor_type_0x1010),
                NETTOUINT(tmpDescriptor.plcDescriptor.last_descriptor_id),
                NETTOUINT(tmpDescriptor.plcDescriptor.plc_description_id),
                NETTOUINT(tmpDescriptor.plcDescriptor.plc_version_descriptor_id),
                NETTOUINT(tmpDescriptor.plcDescriptor.plc_author_descriptor_id),
                NETTOUINT(tmpDescriptor.plcDescriptor.descriptor_slot_size),
                numOfDevicesInPLC,
                NETTOUINT(tmpDescriptor.plcDescriptor.plc_flags),
                tmpDescriptor.plcDescriptor.name_of_plc);
#ifdef motorMessageTextString
      (void)setStringParam(motorMessageText_, tmpDescriptor.plcDescriptor.name_of_plc);
#endif
      break;

    case 0x2014:
      {
        unsigned string_description_id = NETTOUINT(tmpDescriptor.deviceDescriptor.string_description_id);
        unsigned target_param_descriptor_id = NETTOUINT(tmpDescriptor.deviceDescriptor.target_param_descriptor_id);
        unsigned auxbits_bitfield_flag_descriptor_id = NETTOUINT(tmpDescriptor.deviceDescriptor.auxbits_bitfield_flag_descriptor_id);
        unsigned parameters_descriptor_id = NETTOUINT(tmpDescriptor.deviceDescriptor.parameters_descriptor_id);
        unsigned enum_errorId_descriptor_id = NETTOUINT(tmpDescriptor.deviceDescriptor.enum_errorId_descriptor_id);
        unsigned type_code = NETTOUINT(tmpDescriptor.deviceDescriptor.type_code);
        unsigned device_offset = NETTOUINT(tmpDescriptor.deviceDescriptor.device_offset);
        unsigned device_flags = NETTOUINT(tmpDescriptor.deviceDescriptor.device_flags);
        asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                  "%s%s descID=0x%04X type=0x%X deviceDescriptor prev=0x%04X string=0x%04X target=0x%04X auxbits=0x%04X paramters=0x%04X enum_errorID=0x%04X type_code=0x%04X offset=%u flags=0x%x name=\"%s\"\n",
                  modNamEMC, c_function_name, descID,
                  NETTOUINT(tmpDescriptor.deviceDescriptor.descriptor_type_0x2014),
                  NETTOUINT(tmpDescriptor.deviceDescriptor.prev_descriptor_id),
                  string_description_id,
                  target_param_descriptor_id,
                  auxbits_bitfield_flag_descriptor_id,
                  parameters_descriptor_id,
                  enum_errorId_descriptor_id,
                  type_code,
                  device_offset,
                  device_flags,
                  tmpDescriptor.deviceDescriptor.device_name);
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
                                    tmpDescriptor.deviceDescriptor.device_name);
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
                NETTOUINT(tmpDescriptor.stringDescriptor.descriptor_type_0x3004),
                NETTOUINT(tmpDescriptor.stringDescriptor.prev_descriptor_id),
                tmpDescriptor.stringDescriptor.utf8_string);
      break;

    case 0x4006:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X enumDescriptor prev=0x%04X enum_value=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.enumDescriptor.descriptor_type_0x4006),
                NETTOUINT(tmpDescriptor.enumDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.enumDescriptor.enum_value),
                tmpDescriptor.enumDescriptor.enum_name);
      break;
    case 0x5008:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X bitfieldDescriptor prev=0x%04X last=0x%04X lowest=%u width=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.bitfieldDescriptor.descriptor_type_0x5008),
                NETTOUINT(tmpDescriptor.bitfieldDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.bitfieldDescriptor.last_descriptor_id),
                NETTOUINT(tmpDescriptor.bitfieldDescriptor.lowest_bit),
                NETTOUINT(tmpDescriptor.bitfieldDescriptor.bit_width),
                tmpDescriptor.bitfieldDescriptor.bitfield_name);
      break;
    case 0x5105:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X flagDescriptor prev=0x%04X bit_number=%u utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.flagDescriptor.descriptor_type_0x5105),
                NETTOUINT(tmpDescriptor.flagDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.flagDescriptor.bit_number),
                tmpDescriptor.flagDescriptor.flag_name);
      break;
    case 0x6114:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X parameterDescriptor prev=0x%04X string=0x%04X index=%u type=0x%04X unit=0x%x min=%f max=%f utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.parameterDescriptor.descriptor_type_0x6114),
                NETTOUINT(tmpDescriptor.parameterDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.parameterDescriptor.string_description_id),
                NETTOUINT(tmpDescriptor.parameterDescriptor.parameter_index),
                NETTOUINT(tmpDescriptor.parameterDescriptor.parameter_type),
                NETTOUINT(tmpDescriptor.parameterDescriptor.unit),
                NETTODOUBLE(tmpDescriptor.parameterDescriptor.min_value),
                NETTODOUBLE(tmpDescriptor.parameterDescriptor.max_value),
                tmpDescriptor.parameterDescriptor.parameter_name);
      break;
    case 0x620e:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X enumparamDescriptor prev=0x%04X string=0x%04X read=0x%04X write=0x%04X index=0x%04X type=0x%04X utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.enumparamDescriptor.descriptor_type_0x620e),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.string_description_id),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_read_id),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_write_id),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_index),
                NETTOUINT(tmpDescriptor.enumparamDescriptor.enumparam_type),
                tmpDescriptor.enumparamDescriptor.enumparam_name);
      break;
    case 0x680c:
    case 0x680e:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s functionDescriptor descID=0x%04X type=0x%X prev=0x%04X string=0x%04X argument=0x%04X result=0x%04X index=0x%04X flags=0x%x utf8_string=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.functionDescriptor.descriptor_type_0x680e),
                NETTOUINT(tmpDescriptor.functionDescriptor.prev_descriptor_id),
                NETTOUINT(tmpDescriptor.functionDescriptor.string_description_id),
                NETTOUINT(tmpDescriptor.functionDescriptor.function_argument_id),
                NETTOUINT(tmpDescriptor.functionDescriptor.function_result_id),
                NETTOUINT(tmpDescriptor.functionDescriptor.function_index),
                NETTOUINT(tmpDescriptor.functionDescriptor.function_flags),
                tmpDescriptor.functionDescriptor.function_name);
      break;
    case 0x7FFF:
      asynPrint(pasynUserController_, ASYN_TRACE_INFO,
                "%s%s descID=0x%04X type=0x%X debugDescriptor cycleCounter=0x%04X lengthOfDebugDescriptor=%u name=\"%s\"\n",
                modNamEMC, c_function_name, descID,
                NETTOUINT(tmpDescriptor.debugDescriptor.descriptor_type_0xFFFF),
                NETTOUINT(tmpDescriptor.debugDescriptor.cycleCounter),
                NETTOUINT(tmpDescriptor.debugDescriptor.lengthOfDebugDescriptor),
                tmpDescriptor.debugDescriptor.message);
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

