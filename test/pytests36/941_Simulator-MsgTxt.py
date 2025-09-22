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
sevrNone = 0
sevrMinor = 1
sevrMajor = 2
sevrInvalid = 3

NONE_ALARM = 0
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


#   Test case number, errorId, autoPower MsgTxt
#   tc_no            errId  au  MsgTxt
idleWarnTCs = [
    # 1..7: Not homed
    # 1: not homed, not enabled, autopower off
    (1, "0x10800000", 0x0000, 0, "W: PowerOff"),
    # 2: not homed, enabled, autopower off
    (2, "0x10C00000", 0x0000, 0, "W: Axis not homed"),
    # 3: not homed, not enabled, autopower on
    (3, "0x10800000", 0x0000, 1, "W: Axis not homed"),
    # 4: not homed, enabled, autopower on
    (4, "0x10C00000", 0x0000, 1, "W: Axis not homed"),
    # 5: not homed, not enabled, autopower off, error Id
    (5, "0x10800000", 0x4460, 0, "W: PowerOff"),
    # 6: not homed, enabled, autopower off, error Id
    (6, "0x10C00000", 0x4460, 0, "W: Axis not homed"),
    # 7: not homed, not enabled, autopower on, error Id
    (7, "0x10800000", 0x4460, 1, "W: Axis not homed"),
    # 8: not homed, enabled, autopower on, error Id
    (8, "0x10C00000", 0x4460, 1, "W: Axis not homed"),
    # 9..10 homed, not enabled/enabled, autopower off, error Id
    (9, "0x10000000", 0x4460, 0, "W: PowerOff"),
    (10, "0x10400000", 0x4460, 0, "W: Low soft lim 4460"),
    # 11..12 homed, not enabled/enabled, autopower on, error Id
    (11, "0x10000000", 0x4460, 1, "W: Low soft lim 4460"),
    (12, "0x10400000", 0x4460, 1, "W: Low soft lim 4460"),
    # homed, enabled, localmode
    (13, "0x10500000", 0x4460, 0, "W: localMode"),
    (14, "0x10500000", 0x0, 0, "W: localMode"),
    # homed, enabled, InterlockBwd
    (15, "0x10480000", 0x4460, 0, "W: InterlockBwd"),
    (16, "0x10480000", 0x0, 0, "W: InterlockBwd"),
    # homed, enabled, InterlockFwd
    (17, "0x10440000", 0x4460, 0, "W: InterlockFwd"),
    (18, "0x10440000", 0x0, 0, "W: InterlockFwd"),
    # homed, enabled, InterlockBwd, InterlockFwd
    (19, "0x104C0000", 0x4460, 0, "W: InterlockFwdBwd"),
    (20, "0x104C0000", 0x0, 0, "W: InterlockFwdBwd"),
]

errorTCs = [
    # Error state
    (1, "0x80000000", 0x4550, 0, "E: Follw errpos 4550"),
    (2, "0x80400000", 0x4550, 0, "E: Follw errpos 4550"),
    (3, "0x80000000", 0x4550, 1, "E: Follw errpos 4550"),
    (4, "0x80400000", 0x4550, 1, "E: Follw errpos 4550"),
    (5, "0x80000000", 0x4550, 0, "E: Follw errpos 4550"),
    (6, "0x80400000", 0x4550, 0, "E: Follw errpos 4550"),
    # Error state, no error Id, look at the reason bits.
    # Only bit should be set
    (7, "0x81400000", 0x0, 0, "E: Inhibit"),
    (8, "0x82400000", 0x0, 0, "E: Timeout"),
    (9, "0x84400000", 0x0, 0, "E: LowLimit"),
    (10, "0x88400000", 0x0, 0, "E: HighLimit"),
    # Error state, no error Id, 2 reason bits set. Hex code of reason bits
    (11, "0x83400000", 0x0, 0, "E: Error (0x3)"),
    # Error state, no error Id, all reason bits set. Hex code of reason bits
    (12, "0x8F400000", 0x0, 0, "E: Error (0xF)"),
]

resetTCs = [
    # Reset state
    (1, "0x00000000", 0x0, 0, "W: AxisRESET"),
    # Reset state. errorID should be ignored
    (2, "0x00000000", 0x4550, 0, "W: AxisRESET"),
]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeBitsReadMsgTxt(
    self,
    tc_no=0,
    statusReasonAux=None,
    errorId=-1,
    pwrAuto=-1,
    expMsgTxt="undef",
    expSevr=None,
    expStat=None,
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert statusReasonAux is not None
    assert errorId != -1
    assert pwrAuto != -1
    assert expMsgTxt != "undef"
    assert expSevr is not None
    assert expStat is not None

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
        actSevr = int(self.axisCom.get("-MsgTxt.SEVR"))
        actStat = int(self.axisCom.get("-MsgTxt.STAT"))
        if actMsgTxt == " ":
            actMsgTxt = ""
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} maxTime={maxTime:.2f} expMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r} expSevr={expSevr!r} actSevr={actSevr!r} expStat={expStat!r} actStat={actStat!r}"
        )

        if actMsgTxt == expMsgTxt and actSevr == expSevr and actStat == expStat:
            passed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} passed={passed} statusReasonAux={statusReasonAux} errorId={errorId:x} pwrAuto={pwrAuto} ampliexpMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r} expSevr={expSevr!r} actSevr={actSevr!r} expStat={expStat!r} actStat={actStat!r}"
    )
    self.axisCom.put("-PwrAuto", oldPwrAuto)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    return passed


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

    # test cases. Ordered after the state from above: reset..error
    def test_TC_94101(self):
        passed = True
        for tc in resetTCs:
            tc_no = 94101000 + tc[0]
            statusReasonAux = tc[1]
            errorId = tc[2]
            pwrAuto = tc[3]
            expMsgTxt = tc[4]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAuxIdlePowerOn,
                errorId=0,
                pwrAuto=1,
                expMsgTxt="",
                expSevr=sevrNone,
                expStat=NONE_ALARM,
            )
            tc_no = 94101100 + tc[0]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                pwrAuto=pwrAuto,
                expMsgTxt=expMsgTxt,
                expSevr=sevrMinor,
                expStat=UDF_ALARM,
            )

        assert passed

    def test_TC_94102(self):
        passed = True
        for tc in idleWarnTCs:
            tc_no = 94102000 + tc[0]
            statusReasonAux = tc[1]
            errorId = tc[2]
            pwrAuto = tc[3]
            expMsgTxt = tc[4]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAuxIdlePowerOn,
                errorId=0,
                pwrAuto=1,
                expMsgTxt="",
                expSevr=sevrNone,
                expStat=NONE_ALARM,
            )
            tc_no = 94102100 + tc[0]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                pwrAuto=pwrAuto,
                expMsgTxt=expMsgTxt,
                expSevr=sevrMinor,
                expStat=STATE_ALARM,
            )
            # turn idle into warning. Both give the same result.
            statusReasonAux = "0x3" + (tc[1])[3:]
            tc_no = 94102200 + tc[0]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAuxIdlePowerOn,
                errorId=0,
                pwrAuto=1,
                expMsgTxt="",
                expSevr=sevrNone,
                expStat=NONE_ALARM,
            )
            tc_no = 94102300 + tc[0]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                pwrAuto=pwrAuto,
                expMsgTxt=expMsgTxt,
                expSevr=sevrMinor,
                expStat=STATE_ALARM,
            )

        assert passed

    def test_TC_94103(self):
        passed = True
        for tc in errorTCs:
            tc_no = 94103000 + tc[0]
            statusReasonAux = tc[1]
            errorId = tc[2]
            pwrAuto = tc[3]
            expMsgTxt = tc[4]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAuxIdlePowerOn,
                errorId=0,
                pwrAuto=1,
                expMsgTxt="",
                expSevr=sevrNone,
                expStat=NONE_ALARM,
            )
            tc_no = 94103100 + tc[0]
            passed = passed and writeBitsReadMsgTxt(
                self,
                tc_no=tc_no,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                pwrAuto=pwrAuto,
                expMsgTxt=expMsgTxt,
                expSevr=sevrMajor,
                expStat=STATE_ALARM,
            )

        assert passed

    def test_TC_94199999(self):
        tc_no = 94102
        writeBitsReadMsgTxt(
            self,
            tc_no=tc_no,
            statusReasonAux=statusReasonAuxIdlePowerOn,
            errorId=0,
            pwrAuto=2,
            expMsgTxt="",
            expSevr=sevrNone,
            expStat=NONE_ALARM,
        )
