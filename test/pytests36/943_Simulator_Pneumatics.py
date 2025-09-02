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

# https://github.com/EuropeanSpallationSource/tc_mca_std_lib/blob/master/DUTs/Pneumatics/E_PneumaticAxisErrors.TcDUT
# TYPE E_PneumaticAxisErrors :
# (
#    eNoError := 0,
#    eExtractTimedOut := 1, //Extraction movement not done in expected time
#    eRetractTimedOut := 2, //Retraction movement not done in expected time
#    eNotMovingExtract := 3, //Extract command not executed, cylinder stays in place
#    eNotMovingRetract := 4, //Retract command not executed, cylinder stays in place
#    eNoPSSPermit := 5, //Permit signal lost, and trying to move
#    eRetractInterlocked := 6, //Cylinder interlocked, but given signal to Extend
#    eExtendInterlocked := 7, //Cylinder interlocked, but given signal to Retract
#    eAirPressureErrorLow := 8, //Air pressure too low
#    eAirPressureErrorHigh := 9, //Air pressure too high
#    eNoSignalFromEndSwitchBwd := 10, //Lost signal from the EndSwitchBwd
#    eNoSignalFromEndSwitchFwd := 11 //Lost signal from the EndSwitchFwd
# ):= eNoError;

#   Test case number, error, autoPower, ErrTxt
#   tc_no     e            errId  au   MsgTxt         ErrTxt
errTCs = [
    (943001, "0x80000000", 0x10001, "E: Extract Timeout", "E: Extract Timeout"),
    (943002, "0x80000000", 0x10002, "E: Retract Timeout", "E: Retract Timeout"),
    (943003, "0x80000000", 0x10003, "E: Not Moving Extract", "E: Not Moving Extract"),
    (943004, "0x80000000", 0x10004, "E: Not Moving Retract", "E: Not Moving Retract"),
    (943005, "0x80000000", 0x10005, "E: No PSS Permit", "E: No PSS Permit"),
    (943006, "0x80000000", 0x10006, "E: Retract Interlocked", "E: Retract Interlocked"),
    (943007, "0x80000000", 0x10007, "E: Extract Interlocked", "E: Extract Interlocked"),
    (943008, "0x80000000", 0x10008, "E: Air Pressure Low", "E: Air Pressure Low"),
    (943009, "0x80000000", 0x10009, "E: Air Pressure High", "E: Air Pressure High"),
    (
        943010,
        "0x80000000",
        0x1000A,
        "E: NoSignalEndSwitchBwd",
        "E: NoSignalEndSwitchBwd",
    ),
    (
        943011,
        "0x80000000",
        0x1000B,
        "E: NoSignalEndSwitchFwd",
        "E: NoSignalEndSwitchFwd",
    ),
]

aux07TCs = [
    #    (943020, "0x10000000", "E: Extract Timeout", "E: Extract Timeout"),
    (943021, "0x10000001", "Closed"),
    (943022, "0x10000002", "Closing"),
    (943023, "0x10000004", "InTheMiddle"),
    (943024, "0x10000008", "Opening"),
    (943025, "0x10000010", "Opened"),
]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeBitsReadMsgTxtErrTxt(
    self,
    tc_no=0,
    statusReasonAux=None,
    errorId=-1,
    expMsgTxt="undef",
    expErrTxt="undef",
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert statusReasonAux is not None
    assert errorId != -1
    assert expMsgTxt != "undef"
    assert expErrTxt != "undef"

    maxTime = 5  # 5 seconds maximum to let read only parameters ripple through
    passed = False
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", statusReasonAux)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)

    self.axisMr.setValueOnSimulator(tc_no, "nErrorId", errorId)
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
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} passed={passed} statusReasonAux={statusReasonAux} errorId={errorId:x} ampliexpMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r} expErrTxt='{expErrTxt}' actErrTxt={actErrTxt!r}"
    )
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


def writeBitsRead07Txt(
    self,
    tc_no=0,
    statusReasonAux=None,
    exp07Txt="undef",
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert statusReasonAux is not None
    assert exp07Txt != "undef"

    maxTime = 2  # 2 seconds maximum to let ripple through
    passed = False
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", statusReasonAux)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)

    while maxTime > 0:
        act07Num = int(self.axisCom.get("AuxBits07"))
        act07Txt = str(self.axisCom.get("AuxBits07"))
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} maxTime={maxTime:.2f} exp07Txt='{exp07Txt}' act07Txt={act07Txt} act07Num={act07Num:d}"
        )

        if act07Txt == exp07Txt:
            passed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} passed={passed} statusReasonAux={statusReasonAux} exp07Txt='{exp07Txt}' act07Txt={act07Txt!r}"
    )
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_943000(self):
        for tc in errTCs:
            tc_no = tc[0]
            statusReasonAux = tc[1]
            errorId = tc[2]
            expMsgTxt = tc[3]
            expErrTxt = tc[4]
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10,
                statusReasonAux=idxStatusCodeIDLE,
                errorId=0,
                expMsgTxt="",
                expErrTxt="",
            )
            writeBitsReadMsgTxtErrTxt(
                self,
                tc_no=tc_no * 10 + 1,
                statusReasonAux=statusReasonAux,
                errorId=errorId,
                expMsgTxt=expMsgTxt,
                expErrTxt=expErrTxt,
            )

    def test_TC_943020(self):
        for tc in aux07TCs:
            tc_no = tc[0]
            statusReasonAux = tc[1]
            exp07Txt = tc[2]
            writeBitsRead07Txt(
                self,
                tc_no=tc_no * 10 + 1,
                statusReasonAux=statusReasonAux,
                exp07Txt=exp07Txt,
            )

    def xest_TC_94399999(self):
        tc_no = 94399999
        writeBitsReadMsgTxtErrTxt(
            self,
            tc_no=tc_no,
            statusReasonAux=idxStatusCodeIDLE,
            errorId=0,
            expMsgTxt="",
            expErrTxt="",
        )
