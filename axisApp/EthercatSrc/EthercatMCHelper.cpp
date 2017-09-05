/*
  FILENAME... EthercatMCHelper.cpp
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

const static char *const modulName = "EthercatMCAxis::";

/** Writes a command to the axis, and expects a logical ack from the controller
 * Outdata is in pC_->outString_
 * Indata is in pC_->inString_
 * The communiction is logged ASYN_TRACE_INFO
 *
 * When the communictaion fails ot times out, writeReadOnErrorDisconnect() is called
 */
asynStatus EthercatMCAxis::writeReadACK(void)
{
  asynStatus status = pC_->writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
    {
      const char *semicolon = &pC_->outString_[0];
      unsigned int numOK = 1;
      int res = 1;
      while (semicolon && semicolon[0]) {
        semicolon = strchr(semicolon, ';');
        if (semicolon) {
          numOK++;
          semicolon++;
        }
      }
      switch(numOK) {
        case 1: res = strcmp(pC_->inString_, "OK");  break;
        case 2: res = strcmp(pC_->inString_, "OK;OK");  break;
        case 3: res = strcmp(pC_->inString_, "OK:OK;OK");  break;
        case 4: res = strcmp(pC_->inString_, "OK;OK;OK;OK");  break;
        default:
          ;
      }
      if (res) {
        status = asynError;
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s out=%s in=%s return=%s (%d)\n",
                   modulName,
                  pC_->outString_, pC_->inString_,
                  pasynManager->strStatus(status), (int)status);
        if (!drvlocal.cmdErrorMessage[0]) {
          snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
                   "E: writeReadACK() out=%s in=%s\n",
                   pC_->outString_, pC_->inString_);
          /* The poller co-ordinates the writing into the parameter library */
        }
        return status;
      }
    }
    default:
      break;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s out=%s in=%s status=%s (%d)\n",
             modulName,
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status);
  return status;
}


/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValueOnAxis(const char* var, int value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%d", axisNo_, var, value);
  return writeReadACK();
}

/** Sets an integer or boolean value on an axis, read it back and retry if needed
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
asynStatus EthercatMCAxis::setValueOnAxisVerify(const char *var, const char *rbvar,
                                                int value, unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter <= retryCount) {
    sprintf(pC_->outString_, "Main.M%d.%s=%d;Main.M%d.%s?",
            axisNo_, var, value, axisNo_, rbvar);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
               modulName,
              axisNo_,pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);
    if (status) {
      return status;
    } else {
      int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s nvals=%d command=\"%s\" response=\"%s\"\n",
                   modulName,
                  nvals, pC_->outString_, pC_->inString_);
        return asynError;
      }
      if (status) break;
      if (rbvalue == value) break;
      counter++;
      epicsThreadSleep(.1);
    }
  }
  /* Verification failed.
     Store the error (unless there was an error before) */
  if ((rbvalue != value) && !drvlocal.cmdErrorMessage[0]) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
               "%s setValueOnAxisV(%d) var=%s value=%d rbvalue=%d",
                 modulName, axisNo_,var, value, rbvalue);
      snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage)-1,
               "E: setValueOnAxisV(%s) value=%d rbvalue=%d",
               var, value, rbvalue);

      /* The poller co-ordinates the writing into the parameter library */
  }
  return status;
}

/** Sets a floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValueOnAxis(const char* var, double value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%g", axisNo_, var, value);
  return writeReadACK();
}

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus EthercatMCAxis::setValuesOnAxis(const char* var1, double value1,
                                           const char* var2, double value2)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%g;Main.M%d.%s=%g",
          axisNo_, var1, value1, axisNo_, var2, value2);
  return writeReadACK();
}


int EthercatMCAxis::getMotionAxisID(void)
{
  int ret = drvlocal.dirty.nMotionAxisID;
  if (ret < 0) {
    asynStatus comStatus = getValueFromAxis("nMotionAxisID", &ret);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s getMotionAxisID(%d) comStatus=%d ret=%d\n",
               modulName, axisNo_, (int)comStatus, ret);
    if (comStatus) return -1;
  }

  if (ret >= 0) drvlocal.dirty.nMotionAxisID = ret;

  return ret;
}

asynStatus EthercatMCAxis::setADRValueOnAxis(unsigned adsport,
                                        unsigned indexGroup,
                                        unsigned indexOffset,
                                        int value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
          adsport, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setADRValueOnAxisVerify(unsigned adsport,
                                              unsigned indexGroup,
                                              unsigned indexOffset,
                                              int value,
                                              unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getADRValueFromAxis(adsport, indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setValueOnAxisVerify(%d) out=%s in=%s\n",
               modulName, axisNo_,pC_->outString_, pC_->inString_);
    status = setADRValueOnAxis(adsport, indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::setADRValueOnAxis(unsigned adsport,
                                        unsigned indexGroup,
                                        unsigned indexOffset,
                                        double value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
          adsport, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setADRValueOnAxisVerify(unsigned adsport,
                                              unsigned indexGroup,
                                              unsigned indexOffset,
                                              double value,
                                              unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getADRValueFromAxis(adsport, indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s setValueOnAxisVerify(%d) out=%s in=%s\n",
               modulName, axisNo_, pC_->outString_, pC_->inString_);
    status = setADRValueOnAxis(adsport, indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::getADRValueFromAxis(unsigned adsport,
                                          unsigned indexGroup,
                                          unsigned indexOffset,
                                          int *value)
{
  int res;
  int nvals;
  asynStatus comStatus;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
          adsport, indexGroup + axisID, indexOffset);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

asynStatus EthercatMCAxis::getADRValueFromAxis(unsigned adsport,
                                          unsigned indexGroup,
                                          unsigned indexOffset,
                                          double *value)
{
  double res;
  int nvals;
  asynStatus comStatus;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          adsport, indexGroup + axisID, indexOffset);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}


/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxis::getValueFromAxis(const char* var, int *value)
{
  asynStatus comStatus;
  int res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s command=\"%s\" response=\"%s\"\n",
                 modulName, pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s nvals=%d command=\"%s\" response=\"%s\"\n",
                 modulName, nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  *value = res;
  return asynSuccess;
}

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxis::getADRValuesFromAxisPrint(unsigned adsport,
                                                     unsigned iIndexGroup,
                                                     unsigned iIndexOffset,
                                                     int *iValue,
                                                     unsigned fIndexGroup,
                                                     unsigned fIndexOffset,
                                                     double *fValue)
{
  int iRes;
  int nvals;
  double fRes;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?;ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          adsport, iIndexGroup + axisID, iIndexOffset,
          adsport, fIndexGroup + axisID, fIndexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d;%lf", &iRes, &fRes);
  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s out=%s in=%s iValue=%d fValue=%g\n",
            modulName, pC_->outString_, pC_->inString_, iRes, fRes);

  *iValue = iRes;
  *fValue = fRes;
  return asynSuccess;

}

/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
asynStatus EthercatMCAxis::getValueFromAxis(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Gets a string value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the string result
 *
 */
asynStatus EthercatMCAxis::getStringFromAxis(const char *var, char *value, size_t maxlen)
{
  asynStatus comStatus;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus) return comStatus;

  memcpy(value, pC_->inString_, maxlen);
  return asynSuccess;
}

asynStatus EthercatMCAxis::getValueFromController(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "%s?", var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}


