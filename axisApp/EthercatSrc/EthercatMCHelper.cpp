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
const static unsigned int MINADSPORT = 851; /* something useful */
const static unsigned int MAXADSPORT = 861; /* something useful */

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
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s=%d", drvlocal.adsport, axisNo_, var, value);
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
    sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s=%d;Main.M%d.%s?",
            drvlocal.adsport, axisNo_, var, value, axisNo_, rbvar);
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
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s=%g", drvlocal.adsport, axisNo_, var, value);
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
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s=%g;Main.M%d.%s=%g",
          drvlocal.adsport, axisNo_, var1, value1, axisNo_, var2, value2);
  return writeReadACK();
}


int EthercatMCAxis::getMotionAxisID(void)
{
  int ret = drvlocal.dirty.nMotionAxisID;
  if (ret < 0) {
    asynStatus status;
    unsigned int adsport;
    for (adsport = MINADSPORT; adsport <= MAXADSPORT; adsport++) {
      sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.nMotionAxisID?",
              adsport, axisNo_);
      status = pC_->writeReadOnErrorDisconnect();
      if (status) {
        return -1;
      }
      int res;
      int nvals = sscanf(pC_->inString_, "%d", &res);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "%s nvals=%d command=\"%s\" response=\"%s\"\n",
                  modulName, nvals, pC_->outString_, pC_->inString_);
        continue;
      }
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s out=%s in=%s status=%s (%d) iValue=%d\n",
                modulName,
                pC_->outString_, pC_->inString_,
                pasynManager->strStatus(status), (int)status, res);
      ret = res;
      drvlocal.adsport = adsport;
      break;
    }
  }
  if (ret >= 0) drvlocal.dirty.nMotionAxisID = ret;
  return ret;
}


void EthercatMCAxis::getFeatures(void)
{
  if (!drvlocal.dirty.features) return;
  /* The features we know about */
  const char * const sim_str = "sim";
  const char * const stECMC_str = "ecmc";
  const char * const stV1_str = "stv1";
  asynStatus status = asynSuccess;
  unsigned int adsport;
  for (adsport = MINADSPORT; adsport <= MAXADSPORT; adsport++) {
    sprintf(pC_->outString_, "ADSPORT=%u/.THIS.sFeatures?", adsport);
    pC_->inString_[0] = 0;
    status = pC_->writeReadController();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s out=%s in=%s status=%s (%d)\n",
              modulName, pC_->outString_, pC_->inString_,
              pasynManager->strStatus(status), (int)status);

    if (status) return;

    /* loop through the features */
    char *pFeatures = strdup(pC_->inString_);
    char *pThisFeature = pFeatures;
    char *pNextFeature = pFeatures;

    while (pNextFeature && pNextFeature[0]) {
      pNextFeature = strchr(pNextFeature, ';');
      if (pNextFeature) {
        *pNextFeature = '\0'; /* Terminate */
        pNextFeature++;       /* Jump to (possible) next */
      }
      if (!strcmp(pThisFeature, sim_str)) {
        drvlocal.supported.bSIM = 1;
      } else if (!strcmp(pThisFeature, stECMC_str)) {
        drvlocal.supported.bECMC = 1;
      } else if (!strcmp(pThisFeature, stV1_str)) {
        drvlocal.supported.stAxisStatus_V1 = 1;
      }
      pThisFeature = pNextFeature;
    }
    free(pFeatures);
    if (drvlocal.supported.bSIM ||
        drvlocal.supported.bECMC ||
        drvlocal.supported.stAxisStatus_V1) {
      /* Found something useful on this adsport */
      drvlocal.dirty.features = 0;
      return;
    }
  }
}


asynStatus EthercatMCAxis::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             int value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
          501, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   int value,
                                                   unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             double value)
{
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g",
          501, indexGroup + axisID, indexOffset, value);
  return writeReadACK();
}

asynStatus EthercatMCAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   double value,
                                                   unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getSAFValueFromAxisPrint(indexGroup, indexOffset, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus EthercatMCAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    int *value)
{
  int res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?",
          501, indexGroup + axisID, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s out=%s in=%s status=%s (%d) iValue=%d\n",
            modulName,
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status, res);
  *value = res;
  return asynSuccess;
}

asynStatus EthercatMCAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    double *value)
{
  double res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID < 0) return asynError;
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
          501, indexGroup + axisID, indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s nvals=%d command=\"%s\" response=\"%s\"\n",
               modulName, nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s out=%s in=%s status=%s (%d) fValue=%g\n",
            modulName,
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status, res);

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
  asynStatus status;
  int res;
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s?", drvlocal.adsport, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
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
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s out=%s in=%s status=%s (%d) iValue=%d\n",
            modulName,
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status, res);

  *value = res;
  return asynSuccess;
}

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus EthercatMCAxis::getSAFValuesFromAxisPrint(unsigned iIndexGroup,
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
          501, iIndexGroup + axisID, iIndexOffset,
          501, fIndexGroup + axisID, fIndexOffset);
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
  asynStatus status;
  int nvals;
  double res;
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s?", drvlocal.adsport, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
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
  asynStatus status;
  sprintf(pC_->outString_, "ADSPORT=%u/Main.M%d.%s?", drvlocal.adsport, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;

  memcpy(value, pC_->inString_, maxlen);
  return asynSuccess;
}

asynStatus EthercatMCAxis::getValueFromController(const char* var, double *value)
{
  asynStatus status;
  int nvals;
  double res;
  sprintf(pC_->outString_, "%s?", var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status)
    return status;
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


asynStatus EthercatMCAxis::readConfigFile(void)
{
  const char *setRaw_str = "setRaw ";
  const char *setSim_str = "setSim ";
  const char *setADRinteger_str = "setADRinteger ";
  const char *setADRdouble_str  = "setADRdouble ";
  FILE *fp;
  char *ret = &pC_->outString_[0];
  int line_no = 0;
  asynStatus status = asynSuccess;
  const char *errorTxt = NULL;
  /* no config file, or successfully uploaded : return */
  if (!drvlocal.cfgfileStr) {
    drvlocal.dirty.readConfigFile = 0;
    return asynSuccess;
  }
  if (!drvlocal.dirty.readConfigFile) return asynSuccess;

  fp = fopen(drvlocal.cfgfileStr, "r");
  if (!fp) {
    int saved_errno = errno;
    char cwdbuf[4096];
    char errbuf[4196];

    char *mypwd = getcwd(cwdbuf, sizeof(cwdbuf));
    snprintf(errbuf, sizeof(errbuf)-1,
             "E: readConfigFile: %s\n%s/%s",
             strerror(saved_errno),
             mypwd ? mypwd : "",
             drvlocal.cfgfileStr);
    updateMsgTxtFromDriver(errbuf);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s (%d)%s\n", modulName, axisNo_, errbuf);
    return asynError;
  }
  while (ret && !status && !errorTxt) {
    char rdbuf[256];
    size_t i;
    size_t len;
    int nvals = 0;

    line_no++;
    ret = fgets(rdbuf, sizeof(rdbuf), fp);
    if (!ret) break;    /* end of file or error */
    len = strlen(ret);
    if (!len) continue; /* empty line, no LF */
    for (i=0; i < len; i++) {
      /* No LF, no CR , no ctrl characters, */
      if (rdbuf[i] < 32) rdbuf[i] = 0;
    }
    len = strlen(ret);
    if (!len) continue; /* empty line with LF */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "%s readConfigFile %s:%u %s\n",
              modulName,
              drvlocal.cfgfileStr, line_no, rdbuf);

    if (rdbuf[0] == '#') {
      continue; /*  Comment line */
    } else if (!strncmp(setRaw_str, rdbuf, strlen(setRaw_str))) {
      const char *cfg_txt_p = &rdbuf[strlen(setRaw_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;

      snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", cfg_txt_p);
      status = writeReadACK();
    } else if (!strncmp(setSim_str, rdbuf, strlen(setSim_str))) {
      if (drvlocal.supported.bSIM) {
        const char *cfg_txt_p = &rdbuf[strlen(setRaw_str)];
        while (*cfg_txt_p == ' ') cfg_txt_p++;

        snprintf(pC_->outString_, sizeof(pC_->outString_),
                 "Sim.M%d.%s", axisNo_, cfg_txt_p);
        status = writeReadACK();
      }
    } else if (!strncmp(setADRinteger_str, rdbuf, strlen(setADRinteger_str))) {
      unsigned indexGroup;
      unsigned indexOffset;
      int value;
      const char *cfg_txt_p = &rdbuf[strlen(setADRinteger_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;
      nvals = sscanf(cfg_txt_p, "%x %x %d",
                     &indexGroup, &indexOffset, &value);
      if (nvals == 3) {
        status = setSAFValueOnAxisVerify(indexGroup, indexOffset, value, 1);
      } else {
        errorTxt = "Need 4 values";
      }
    } else if (!strncmp(setADRdouble_str, rdbuf, strlen(setADRdouble_str))) {
      unsigned indexGroup;
      unsigned indexOffset;
      double value;
      const char *cfg_txt_p = &rdbuf[strlen(setADRdouble_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;
      nvals = sscanf(cfg_txt_p, "%x %x %lf",
                     &indexGroup, &indexOffset, &value);
      if (nvals == 3) {
        status = setSAFValueOnAxisVerify(indexGroup, indexOffset,
                                         value, 1);
      } else {
        errorTxt = "Need 4 values";
      }
    } else {
      errorTxt = "Illegal command";
    }
    if (status || errorTxt) {
      char errbuf[256];
      errbuf[sizeof(errbuf)-1] = 0;
      if (status) {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "E: %s:%d out=%s\nin=%s",
                 drvlocal.cfgfileStr, line_no, pC_->outString_, pC_->inString_);
      } else {
        snprintf(errbuf, sizeof(errbuf)-1,
                 "E: %s:%d \"%s\"\n%s",
                 drvlocal.cfgfileStr, line_no, rdbuf, errorTxt);
      }

      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s readConfigFile %s\n", modulName, errbuf);
      updateMsgTxtFromDriver(errbuf);
    }
  } /* while */

  if (ferror(fp) || status || errorTxt) {
    if (ferror(fp)) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "%s readConfigFile ferror (%s)\n",
                 modulName,
                drvlocal.cfgfileStr);
    }
    fclose(fp);
    return asynError;
  }

  drvlocal.dirty.readConfigFile = 0;
  return asynSuccess;
}
