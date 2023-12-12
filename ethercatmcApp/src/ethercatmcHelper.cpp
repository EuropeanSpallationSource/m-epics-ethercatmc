/*
  FILENAME... ethercatmcHelper.cpp
*/

#include <epicsThread.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ethercatmcAxis.h"
#include "ethercatmcController.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO 0x0040
#endif

asynStatus ethercatmcAxis::writeReadControllerPrint(int traceMask) {
  asynStatus status = pC_->writeReadOnErrorDisconnect();
  asynPrint(pC_->pasynUserController_, traceMask,
            "%sout=%s in=%s status=%s (%d)\n", modNamEMC, pC_->outString_,
            pC_->inString_, ethercatmcstrStatus(status), (int)status);
  return status;
}

asynStatus ethercatmcAxis::writeReadControllerPrint(void) {
  return writeReadControllerPrint(ASYN_TRACE_INFO);
}

/** Sets an integer or boolean value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (integer) variable to be updated
 *
 */
asynStatus ethercatmcAxis::setValueOnAxis(const char *var, int value) {
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s=%d",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Sets an integer or boolean value on an axis, read it back and retry if
 * needed the values in the controller must be updated \param[in] name of the
 * variable to be updated \param[in] name of the variable where we can read back
 * \param[in] value the (integer) variable to be updated
 * \param[in] number of retries
 */
asynStatus ethercatmcAxis::setValueOnAxisVerify(const char *var,
                                                const char *rbvar, int value,
                                                unsigned int retryCount) {
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter <= retryCount) {
    snprintf(pC_->outString_, sizeof(pC_->outString_),
             "%sMain.M%d.%s=%d;%sMain.M%d.%s?", drvlocal.adsport_str, axisNo_,
             var, value, drvlocal.adsport_str, axisNo_, rbvar);
    status = pC_->writeReadOnErrorDisconnect();
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%ssetValueOnAxisVerify(%d) out=%s in=%s status=%s (%d)\n",
              modNamEMC, axisNo_, pC_->outString_, pC_->inString_,
              ethercatmcstrStatus(status), (int)status);
    if (status) {
      return status;
    } else {
      int nvals = sscanf(pC_->inString_, "OK;%d", &rbvalue);
      if (nvals != 1) {
        asynPrint(pC_->pasynUserController_,
                  ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                  "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC,
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
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%ssetValueOnAxisV(%d) var=%s value=%d rbvalue=%d", modNamEMC,
              axisNo_, var, value, rbvalue);
    snprintf(drvlocal.cmdErrorMessage, sizeof(drvlocal.cmdErrorMessage) - 1,
             "E: setValueOnAxisV(%s) value=%d rbvalue=%d", var, value, rbvalue);

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
asynStatus ethercatmcAxis::setValueOnAxis(const char *var, double value) {
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s=%g",
           drvlocal.adsport_str, axisNo_, var, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

/** Sets 2 floating point value on an axis
 * the values in the controller must be updated
 * \param[in] name of the variable to be updated
 * \param[in] value the (floating point) variable to be updated
 *
 */
asynStatus ethercatmcAxis::setValuesOnAxis(const char *var1, double value1,
                                           const char *var2, double value2) {
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "%sMain.M%d.%s=%g;%sMain.M%d.%s=%g", drvlocal.adsport_str, axisNo_,
           var1, value1, drvlocal.adsport_str, axisNo_, var2, value2);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

int ethercatmcAxis::getMotionAxisID(void) {
  int ret = drvlocal.dirty.nMotionAxisID;
  if (ret == -1) {
    int res = -3;
    asynStatus status;
    static const unsigned adsports[] = {0, 852, 851, 853};
    unsigned adsport_idx;
    ret = -2;
    for (adsport_idx = 0; adsport_idx < sizeof(adsports) / sizeof(adsports[0]);
         adsport_idx++) {
      unsigned adsport = adsports[adsport_idx];
      if (!adsport) {
        adsport = drvlocal.adsPort;
      }
      if (adsport) {
        /* Save adsport_str for the poller */
        snprintf(drvlocal.adsport_str, sizeof(drvlocal.adsport_str),
                 "ADSPORT=%u/", adsport);
      }
      snprintf(pC_->outString_, sizeof(pC_->outString_),
               "%sMain.M%d.nMotionAxisID?", drvlocal.adsport_str, axisNo_);
      status = pC_->writeReadOnErrorDisconnect();
      if (status) {
        return -1;
      }
      int nvals = sscanf(pC_->inString_, "%d", &res);
      if (nvals != 1) {
        asynPrint(
            pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
            "%soldret=%d nvals=%d command=\"%s\" response=\"%s\" res=%d\n",
            modNamEMC, ret, nvals, pC_->outString_, pC_->inString_, res);
        continue;
      }
      ret = res;
      break;
    }
    if (ret != -1) drvlocal.dirty.nMotionAxisID = ret;
  }
  return ret;
}

asynStatus ethercatmcAxis::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset, int value) {
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d", 501, indexGroup + axisID,
           indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus ethercatmcAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   int value,
                                                   unsigned int retryCount) {
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status =
        getSAFValueFromAxisPrint(indexGroup, indexOffset, "value=", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus ethercatmcAxis::setSAFValueOnAxis(unsigned indexGroup,
                                             unsigned indexOffset,
                                             double value) {
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%g", 501, indexGroup + axisID,
           indexOffset, value);
  return pC_->writeReadACK(ASYN_TRACE_INFO);
}

asynStatus ethercatmcAxis::setSAFValueOnAxisVerify(unsigned indexGroup,
                                                   unsigned indexOffset,
                                                   double value,
                                                   unsigned int retryCount) {
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  double rbvalue = 0 - value;
  while (counter < retryCount) {
    status =
        getSAFValueFromAxisPrint(indexGroup, indexOffset, "value", &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setSAFValueOnAxis(indexGroup, indexOffset, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

asynStatus ethercatmcAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    int *value) {
  int res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?", 501, indexGroup + axisID,
           indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%d", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
              pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d\n", modNamEMC, pC_->outString_,
            pC_->inString_, name, res);
  *value = res;
  return asynSuccess;
}

asynStatus ethercatmcAxis::getSAFValueFromAxisPrint(unsigned indexGroup,
                                                    unsigned indexOffset,
                                                    const char *name,
                                                    double *value) {
  double res;
  int nvals;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=%u/.ADR.16#%X,16#%X,8,5?", 501, indexGroup + axisID,
           indexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
              pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%g\n", modNamEMC, pC_->outString_,
            pC_->inString_, name, res);
  *value = res;
  return asynSuccess;
}

/** Gets an integer or boolean value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus ethercatmcAxis::getValueFromAxis(const char *var, int *value) {
  asynStatus status;
  int res;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_,
                ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                "%scommand=\"%s\" response=\"%s\"\n", modNamEMC,
                pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_,
                ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
                pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s status=%s (%d) iValue=%d\n", modNamEMC,
            pC_->outString_, pC_->inString_, ethercatmcstrStatus(status),
            (int)status, res);

  *value = res;
  return asynSuccess;
}

/** Gets an integer (or boolean) and a double value from an axis and print
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the integer result
 *
 */
asynStatus ethercatmcAxis::getSAFValuesFromAxisPrint(
    unsigned iIndexGroup, unsigned iIndexOffset, const char *iname, int *iValue,
    unsigned fIndexGroup, unsigned fIndexOffset, const char *fname,
    double *fValue) {
  int iRes;
  int nvals;
  double fRes;
  asynStatus status;
  int axisID = getMotionAxisID();
  if (axisID <= 0) return asynError;
  snprintf(pC_->outString_, sizeof(pC_->outString_),
           "ADSPORT=%u/.ADR.16#%X,16#%X,2,2?;ADSPORT=%u/.ADR.16#%X,16#%X,8,5?",
           501, iIndexGroup + axisID, iIndexOffset, 501, fIndexGroup + axisID,
           fIndexOffset);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%d;%lf", &iRes, &fRes);
  if (nvals != 2) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
              pC_->outString_, pC_->inString_);
    return asynError;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%sout=%s in=%s %s=%d %s=%g\n", modNamEMC, pC_->outString_,
            pC_->inString_, iname, iRes, fname, fRes);

  *iValue = iRes;
  *fValue = fRes;
  return asynSuccess;
}

/** Gets a floating point value from an axis
 * \param[in] name of the variable to be retrieved
 * \param[in] pointer to the double result
 *
 */
asynStatus ethercatmcAxis::getValueFromAxis(const char *var, double *value) {
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
              pC_->outString_, pC_->inString_);
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
asynStatus ethercatmcAxis::getStringFromAxis(const char *var, char *value,
                                             size_t maxlen) {
  asynStatus status;
  value[0] = '\0'; /* Always have a valid string */
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%sMain.M%d.%s?",
           drvlocal.adsport_str, axisNo_, var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  memcpy(value, pC_->inString_, maxlen);

  return asynSuccess;
}

asynStatus ethercatmcAxis::getValueFromController(const char *var,
                                                  double *value) {
  asynStatus status;
  int nvals;
  double res;
  snprintf(pC_->outString_, sizeof(pC_->outString_), "%s?", var);
  status = pC_->writeReadOnErrorDisconnect();
  if (status) return status;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%snvals=%d command=\"%s\" response=\"%s\"\n", modNamEMC, nvals,
              pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

asynStatus ethercatmcAxis::readConfigLine(const char *line,
                                          const char **errorTxt_p) {
  const char *setRaw_str = "setRaw ";     /* Raw is Raw */
  const char *setValue_str = "setValue "; /* prefixed with ADSPORT */
  const char *setADRinteger_str = "setADRinteger ";
  const char *setADRdouble_str = "setADRdouble ";
  const char *setSim_str = "setSim ";

  asynStatus status = asynError;
  const char *errorTxt = NULL;

  while (*line == ' ') line++;
  if (line[0] == '#') {
    /*  Comment line */
    return asynSuccess;
  }

  if (!strncmp(setRaw_str, line, strlen(setRaw_str))) {
    const char *cfg_txt_p = &line[strlen(setRaw_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;

    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s", cfg_txt_p);
    status = pC_->writeReadACK(ASYN_TRACE_INFO);
  } else if (!strncmp(setValue_str, line, strlen(setValue_str))) {
    const char *cfg_txt_p = &line[strlen(setValue_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;

    snprintf(pC_->outString_, sizeof(pC_->outString_), "%s%s",
             drvlocal.adsport_str, cfg_txt_p);
    status = pC_->writeReadACK(ASYN_TRACE_INFO);
  } else if (!strncmp(setSim_str, line, strlen(setSim_str))) {
    if (pC_->features_ & FEATURE_BITS_SIM) {
      const char *cfg_txt_p = &line[strlen(setRaw_str)];
      while (*cfg_txt_p == ' ') cfg_txt_p++;

      snprintf(pC_->outString_, sizeof(pC_->outString_), "Sim.M%d.%s", axisNo_,
               cfg_txt_p);
      status = pC_->writeReadACK(ASYN_TRACE_INFO);
    } else {
      status = asynSuccess;
    }
  } else if (!strncmp(setADRinteger_str, line, strlen(setADRinteger_str))) {
    unsigned indexGroup;
    unsigned indexOffset;
    int value;
    int nvals = 0;
    const char *cfg_txt_p = &line[strlen(setADRinteger_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;
    nvals = sscanf(cfg_txt_p, "%x %x %d", &indexGroup, &indexOffset, &value);
    if (nvals == 3) {
      status = setSAFValueOnAxisVerify(indexGroup, indexOffset, value, 1);
    } else {
      errorTxt = "Need 4 values";
    }
  } else if (!strncmp(setADRdouble_str, line, strlen(setADRdouble_str))) {
    unsigned indexGroup;
    unsigned indexOffset;
    double value;
    int nvals = 0;
    const char *cfg_txt_p = &line[strlen(setADRdouble_str)];
    while (*cfg_txt_p == ' ') cfg_txt_p++;
    nvals = sscanf(cfg_txt_p, "%x %x %lf", &indexGroup, &indexOffset, &value);
    if (nvals == 3) {
      status = setSAFValueOnAxisVerify(indexGroup, indexOffset, value, 1);
    } else {
      errorTxt = "Need 4 values";
    }
  } else {
    errorTxt = "Illegal command";
  }
  if (errorTxt_p && errorTxt) {
    *errorTxt_p = errorTxt;
  }
  return status;
}

asynStatus ethercatmcAxis::readConfigFile(void) {
  const char *simOnly_str = "simOnly ";
  FILE *fp;
  char *ret = &pC_->outString_[0];
  int line_no = 0;
  asynStatus status = asynSuccess;
  const char *errorTxt = NULL;
  /* no config file, or successfully uploaded : return */
  if (!drvlocal.cfgfileStr) {
    return asynSuccess;
  }
  fp = fopen(drvlocal.cfgfileStr, "r");
  if (!fp) {
    int saved_errno = errno;
    char cwdbuf[4096];
    char errbuf[4196];

    char *mypwd = getcwd(cwdbuf, sizeof(cwdbuf));
    snprintf(errbuf, sizeof(errbuf) - 1, "E: readConfigFile: %s",
             strerror(saved_errno));
    updateMsgTxtFromDriver(errbuf);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "E: readConfigFile: %s %s/%s", strerror(saved_errno),
              mypwd ? mypwd : "", drvlocal.cfgfileStr);
    return asynError;
  }
  while (ret && !status && !errorTxt) {
    char rdbuf[256];
    size_t i;
    size_t len;

    line_no++;
    ret = fgets(rdbuf, sizeof(rdbuf), fp);
    if (!ret) break; /* end of file or error */
    len = strlen(ret);
    if (!len) continue; /* empty line, no LF */
    for (i = 0; i < len; i++) {
      /* No LF, no CR , no ctrl characters, */
      if (rdbuf[i] < 32) rdbuf[i] = 0;
    }
    len = strlen(ret);
    if (!len) continue; /* empty line with LF */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%sreadConfigFile(%d) %s:%u %s\n", modNamEMC, axisNo_,
              drvlocal.cfgfileStr, line_no, rdbuf);

    if (!strncmp(simOnly_str, rdbuf, strlen(simOnly_str))) {
      /* "simOnly " Only for the simulator */
      if (pC_->features_ & FEATURE_BITS_SIM) {
        status = readConfigLine(&rdbuf[strlen(simOnly_str)], &errorTxt);
      }
    } else {
      status = readConfigLine(rdbuf, &errorTxt);
    }

    if (status || errorTxt) {
      char errbuf[256];
      errbuf[sizeof(errbuf) - 1] = 0;

      snprintf(errbuf, sizeof(errbuf) - 1, "E: readConfigFile: line_no=%d",
               line_no);
      if (status) {
        asynPrint(pC_->pasynUserController_,
                  ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
                  "E: %s:%d out=%s\nin=%s", drvlocal.cfgfileStr, line_no,
                  pC_->outString_, pC_->inString_);
      } else {
        asynPrint(pC_->pasynUserController_,
                  ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER, "E: %s:%d \"%s\"\n%s",
                  drvlocal.cfgfileStr, line_no, rdbuf, errorTxt);
      }
      updateMsgTxtFromDriver(errbuf);
    }
  } /* while */

  if (ferror(fp) || status || errorTxt) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR | ASYN_TRACEIO_DRIVER,
              "%sreadConfigFile %sstatus=%d errorTxt=%s (%s)\n", modNamEMC,
              ferror(fp) ? "ferror " : "", (int)status,
              errorTxt ? errorTxt : "", drvlocal.cfgfileStr);
    fclose(fp);
    return asynError;
  }
  return asynSuccess;
}
