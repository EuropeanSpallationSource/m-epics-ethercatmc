#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

filnam = os.path.basename(__file__)[0:3]
###

polltime = 0.05
idxStatusCodeRESET = 0x00000000
idxStatusCodeIDLE = 0x10000000
idxStatusCodePOWEROFF = 0x20000000
idxStatusCodeWARN = 0x30000000
idxStatusCodeERR4 = 0x40000000
idxStatusCodeSTART = 0x50000000
idxStatusCodeBUSY = 0x60000000
idxStatusCodeSTOP = 0x70000000
idxStatusCodeERROR = 0x80000000
idxStatusCodeERR9 = 0x90000000
idxStatusCodeERR10 = 0xA0000000
idxStatusCodeERR11 = 0xB0000000
idxStatusCodeERR12 = 0xC0000000
idxStatusCodeERR13 = 0xD0000000
idxStatusCodeERR14 = 0xE0000000
idxStatusCodeERR15 = 0xF0000000

idxReasonBitHigh = 0x08000000
idxReasonBitLow = 0x04000000
idxReasonBitTimeout = 0x02000000
idxReasonBitInhibit = 0x01000000


# EPICS severities
# hard coded here, inspired by EPICS base
severityNone = 0
severityMinor = 1
severityMajor = 2
severityInvalid = 3


READ_ALARM = 1
WRITE_ALARM = 2
HIHI_ALARM = 3
HIGH_ALARM = 4
LOLO_ALARM = 5
LOW_ALARM = 6
STATE_ALARM = 7
COS_ALARM = 8
COMM_ALARM = 9
TIMEOUT_ALARM = 10
HW_LIMIT_ALARM = 11
CALC_ALARM = 12
SCAN_ALARM = 13
LINK_ALARM = 14
SOFT_ALARM = 15
BAD_SUB_ALARM = 16
UDF_ALARM = 17
DISABLE_ALARM = 18
SIMM_ALARM = 19
READ_ACCESS_ALARM = 20
WRITE_ACCESS_ALARM = 21

# PILS IDLE/WARN/ERROR make sense.
# The rest are illegal
# Error give alarm severity invalid
# But may have differen states, like HIHI
alarmSeverityValuesInt = {
    idxStatusCodeRESET: severityInvalid,
    # idxStatusCodeIDLE
    idxStatusCodePOWEROFF: severityMinor,
    # idxStatusCodeWARN
    idxStatusCodeERR4: severityInvalid,
    idxStatusCodeSTART: severityInvalid,
    idxStatusCodeBUSY: severityInvalid,
    idxStatusCodeSTOP: severityInvalid,
    # idxStatusCodeERROR
    idxStatusCodeERR9: severityInvalid,
    idxStatusCodeERR10: severityInvalid,
    idxStatusCodeERR11: severityInvalid,
    idxStatusCodeERR12: severityInvalid,
    idxStatusCodeERR13: severityInvalid,
    idxStatusCodeERR14: severityInvalid,
    idxStatusCodeERR15: severityInvalid,
}

alarmStateValuesInt = {
    idxStatusCodeRESET: STATE_ALARM,
    # idxStatusCodeIDLE
    idxStatusCodePOWEROFF: STATE_ALARM,
    # idxStatusCodeWARN
    idxStatusCodeERR4: STATE_ALARM,
    idxStatusCodeSTART: STATE_ALARM,
    idxStatusCodeBUSY: STATE_ALARM,
    idxStatusCodeSTOP: STATE_ALARM,
    # idxStatusCodeERROR
    idxStatusCodeERR9: STATE_ALARM,
    idxStatusCodeERR10: STATE_ALARM,
    idxStatusCodeERR11: STATE_ALARM,
    idxStatusCodeERR12: STATE_ALARM,
    idxStatusCodeERR13: STATE_ALARM,
    idxStatusCodeERR14: STATE_ALARM,
    idxStatusCodeERR15: STATE_ALARM,
}

alarmStatSevrValuesInt = {
    idxStatusCodeWARN + idxReasonBitHigh: (severityMinor, HIGH_ALARM),
    idxStatusCodeERROR + idxReasonBitHigh: (severityMajor, HIHI_ALARM),
    idxStatusCodeWARN + idxReasonBitLow: (severityMinor, LOW_ALARM),
    idxStatusCodeERROR + idxReasonBitLow: (severityMajor, LOLO_ALARM),
    idxStatusCodeWARN + idxReasonBitTimeout: (severityMinor, TIMEOUT_ALARM),
    idxStatusCodeERROR + idxReasonBitTimeout: (severityMajor, TIMEOUT_ALARM),
}


#   Test case number, error, homed, autoPower, amplifier on, ErrTxt
#   tc_no     e  errId  hm au on  MsgTxt                ErrTxt
allTCs = [
    (9410001, 0, 0x0000, 0, 0, 0, "E: PowerOff", "E: PowerOff"),
    (9410002, 0, 0x0000, 0, 0, 1, "E: Axis not homed", "E: Axis not homed"),
    (9410003, 0, 0x0000, 0, 1, 0, "E: Axis not homed", "E: Axis not homed"),
    (9410004, 0, 0x0000, 0, 1, 1, "E: Axis not homed", "E: Axis not homed"),
    (9410005, 0, 0x4460, 0, 0, 0, "E: PowerOff", "E: PowerOff"),
    (9410006, 0, 0x4460, 0, 0, 1, "E: Axis not homed", "E: Axis not homed"),
    (9410007, 0, 0x4460, 0, 1, 0, "E: Axis not homed", "E: Axis not homed"),
    (9410008, 0, 0x4460, 0, 1, 1, "E: Axis not homed", "E: Axis not homed"),
    (9410009, 0, 0x4460, 1, 0, 0, "E: PowerOff", "E: PowerOff"),
    (9410010, 0, 0x4460, 1, 0, 1, "W: Low soft lim 4460", ""),
    (9410011, 0, 0x4460, 1, 1, 0, "W: Low soft lim 4460", ""),
    (9410012, 0, 0x4460, 1, 1, 1, "W: Low soft lim 4460", ""),
    (9410020, 1, 0x4550, 0, 0, 0, "E: PowerOff", "E: PowerOff"),
    (9410021, 1, 0x4550, 0, 0, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410022, 1, 0x4550, 0, 1, 0, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410023, 1, 0x4550, 0, 1, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410024, 1, 0x4550, 1, 0, 0, "E: PowerOff", "E: PowerOff"),
    (9410025, 1, 0x4550, 1, 0, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410026, 1, 0x4550, 1, 1, 0, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410027, 1, 0x4550, 1, 1, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeBitsReadMsgTxtErrTxt(
    self,
    tc_no=0,
    axisHomed=-1,
    err=-1,
    errorId=-1,
    pwrAuto=-1,
    amplifierOn=-1,
    msgTxt="undef",
    errTxt="undef",
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert axisHomed >= 0
    assert err != -1
    assert errorId != -1
    assert pwrAuto != -1
    assert amplifierOn != -1
    assert msgTxt != "undef"
    assert errTxt != "undef"

    if amplifierOn == 1:
        amplifierPercent = 100
    else:
        amplifierPercent = 0
    maxTime = 5  # 5 seconds maximum to let read only parameters ripple through
    passed = False
    oldPwrAuto = self.axisCom.get("-PwrAuto")
    self.axisCom.put("-PwrAuto", pwrAuto)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)
    self.axisMr.setValueOnSimulator(tc_no, "bAxisHomed", axisHomed)
    self.axisMr.setValueOnSimulator(tc_no, "bError", err)
    self.axisMr.setValueOnSimulator(tc_no, "nErrorId", errorId)
    self.axisMr.setValueOnSimulator(tc_no, "nAmplifierPercent", amplifierPercent)
    self.axisCom.put(".STUP", 1)
    while maxTime > 0:
        actMsgTxt = self.axisCom.get("-MsgTxt")
        if actMsgTxt == " ":
            actMsgTxt = ""
        actErrTxt = self.axisCom.get("-ErrTxt")
        if actMsgTxt == msgTxt and actErrTxt == errTxt:
            passed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} tc_no={tc_no} passed={passed} axisHomed={axisHomed} err={err} errorId={errorId:x} pwrAuto={pwrAuto} amplifierOn={amplifierOn} msgTxtExp={msgTxt} actMsgTxt={actMsgTxt!r} errTxtExp={errTxt} actErrTxt={actErrTxt!r}"
    )
    self.axisCom.put("-PwrAuto", oldPwrAuto)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


class Test(unittest.TestCase):
    hasROlimit = 0
    drvUseEGU_RB = None
    drvUseEGU = 0
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_94000(self):
        for tc in allTCs:
            tc_no = tc[0]
            err = tc[1]
            errorId = tc[2]
            axisHomed = tc[3]
            pwrAuto = tc[4]
            amplifierOn = tc[5]
            msgTxt = tc[6]
            errTxt = tc[7]
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10,
                axisHomed=1,
                err=0,
                errorId=0,
                pwrAuto=1,
                amplifierOn=1,
                msgTxt="",
                errTxt="",
            )
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10 + 1,
                axisHomed=axisHomed,
                err=err,
                errorId=errorId,
                pwrAuto=pwrAuto,
                amplifierOn=amplifierOn,
                msgTxt=msgTxt,
                errTxt=errTxt,
            )

    def test_TC_94199999(self):
        tc_no = 94102
        writeBitsReadMsgTxtErrTxt(
            self,
            tc_no=tc_no,
            axisHomed=1,
            err=0,
            errorId=0,
            pwrAuto=2,
            amplifierOn=1,
            msgTxt="",
            errTxt="",
        )
