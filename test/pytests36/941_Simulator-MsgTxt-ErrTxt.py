#!/usr/bin/env python
#

import datetime
import inspect
import unittest
import os
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

filnam = os.path.basename(__file__)[0:3]
###

polltime = 0.05
idxStatusCodeRESET = "0x00000000"
idxStatusCodeIDLE = "0x10000000"
statusReasonAuxIdlePowerOn = "0x10400000"
idxStatusCodePOWEROFF = "0x20000000"
idxStatusCodeWARN = "0x30000000"
idxStatusCodeERR4 = "0x40000000"
idxStatusCodeSTART = "0x50000000"
idxStatusCodeBUSY = "0x60000000"
idxStatusCodeSTOP = "0x70000000"
idxStatusCodeERROR = "0x80000000"
idxStatusCodeERR9 = "0x90000000"
idxStatusCodeERR10 = "0xA0000000"
idxStatusCodeERR11 = "0xB0000000"
idxStatusCodeERR12 = "0xC0000000"
idxStatusCodeERR13 = "0xD0000000"
idxStatusCodeERR14 = "0xE0000000"
idxStatusCodeERR15 = "0xF0000000"

idxReasonBitHigh = "0x08000000"
idxReasonBitLow = "0x04000000"
idxReasonBitTimeout = "0x02000000"
idxReasonBitInhibit = "0x01000000"


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


#   Test case number, error, autoPower, ErrTxt
#   tc_no     e  errId  au  MsgTxt                ErrTxt
allTCs = [
    # 1..7: Not homed
    # 1: not homed, not enabled, autopower off
    (9410001, "0x10800000", 0x0000, 0, "E: PowerOff", "E: PowerOff"),
    # 2: not homed, enabled, autopower off
    (9410002, "0x10C00000", 0x0000, 0, "E: Axis not homed", "E: Axis not homed"),
    # 3: not homed, not enabled, autopower on
    (9410003, "0x10800000", 0x0000, 1, "E: Axis not homed", "E: Axis not homed"),
    # 4: not homed, enabled, autopower on
    (9410004, "0x10C00000", 0x0000, 1, "E: Axis not homed", "E: Axis not homed"),
    # 5: not homed, not enabled, autopower off, error Id
    (9410005, "0x10800000", 0x4460, 0, "E: PowerOff", "E: PowerOff"),
    # 6: not homed, enabled, autopower off, error Id
    (9410006, "0x10C00000", 0x4460, 0, "E: Axis not homed", "E: Axis not homed"),
    # 7: not homed, not enabled, autopower on, error Id
    (9410007, "0x10800000", 0x4460, 1, "E: Axis not homed", "E: Axis not homed"),
    # 8: not homed, enabled, autopower on, error Id
    (9410008, "0x10C00000", 0x4460, 1, "E: Axis not homed", "E: Axis not homed"),
    # 9..10 homed, not enabled/enabled, autopower off, error Id
    (9410009, "0x10000000", 0x4460, 0, "E: PowerOff", "E: PowerOff"),
    (9410010, "0x10400000", 0x4460, 0, "W: Low soft lim 4460", ""),
    # 11..12 homed, not enabled/enabled, autopower on, error Id
    (9410011, "0x10000000", 0x4460, 1, "W: Low soft lim 4460", ""),
    (9410012, "0x10400000", 0x4460, 1, "W: Low soft lim 4460", ""),
    # Error state
    (9410020, "0x80000000", 0x4550, 0, "E: PowerOff", "E: PowerOff"),
    (9410021, "0x80400000", 0x4550, 0, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410022, "0x80000000", 0x4550, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410023, "0x80400000", 0x4550, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410024, "0x80000000", 0x4550, 0, "E: PowerOff", "E: PowerOff"),
    (9410025, "0x80400000", 0x4550, 0, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410026, "0x80000000", 0x4550, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
    (9410027, "0x80400000", 0x4550, 1, "E: Follw errpos 4550", "E: Follw errpos 4550"),
]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeBitsReadMsgTxtErrTxt(
    self,
    tc_no=0,
    statusReasonAux=None,
    errorId=-1,
    pwrAuto=-1,
    expMsgTxt="undef",
    expErrTxt="undef",
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert statusReasonAux is not None
    assert errorId != -1
    assert pwrAuto != -1
    assert expMsgTxt != "undef"
    assert expErrTxt != "undef"

    maxTime = 5  # 5 seconds maximum to let read only parameters ripple through
    passed = False
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", statusReasonAux)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)

    self.axisMr.setValueOnSimulator(tc_no, "nErrorId", errorId)
    oldPwrAuto = self.axisCom.get("-PwrAuto")
    self.axisCom.put("-PwrAuto", pwrAuto)
    self.axisCom.put(".STUP", 1)
    while maxTime > 0:
        actMsgTxt = self.axisCom.get("-MsgTxt")
        if actMsgTxt == " ":
            actMsgTxt = ""
        actErrTxt = self.axisCom.get("-ErrTxt")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} maxTime={maxTime:.2f} expMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r} expErrTxt='{expErrTxt}' actErrTxt={actErrTxt!r}"
        )

        if actMsgTxt == expMsgTxt and actErrTxt == expErrTxt:
            passed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} passed={passed} statusReasonAux={statusReasonAux} errorId={errorId:x} pwrAuto={pwrAuto} ampliexpMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r} expErrTxt='{expErrTxt}' actErrTxt={actErrTxt!r}"
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
            statusReasonAux = tc[1]
            errorId = tc[2]
            pwrAuto = tc[3]
            expMsgTxt = tc[4]
            expErrTxt = tc[5]
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10,
                statusReasonAux=statusReasonAuxIdlePowerOn,
                errorId=0,
                pwrAuto=1,
                expMsgTxt="",
                expErrTxt="",
            )
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10 + 1,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                pwrAuto=pwrAuto,
                expMsgTxt=expMsgTxt,
                expErrTxt=expErrTxt,
            )

    def test_TC_94199999(self):
        tc_no = 94102
        writeBitsReadMsgTxtErrTxt(
            self,
            tc_no=tc_no,
            statusReasonAux=statusReasonAuxIdlePowerOn,
            errorId=0,
            pwrAuto=2,
            expMsgTxt="",
            expErrTxt="",
        )
