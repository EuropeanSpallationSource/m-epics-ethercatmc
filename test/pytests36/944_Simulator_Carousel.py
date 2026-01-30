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


aux07TCs = [
    #    (944020, "0x10000000", "E: Extract Timeout", "E: Extract Timeout"),
    (944021, "0x10000001", "OutOfCarousel"),
    (944022, "0x10000002", "Position1"),
    (944023, "0x10000004", "Position2"),
    (944024, "0x10000008", "Position3"),
    (944025, "0x10000010", "Position4"),
    (944026, "0x10000020", "Position5"),
    (944027, "0x10000040", "Position6"),
    (944028, "0x10000080", "Position7"),
    (944029, "0x10000100", "Position8"),
    (944030, "0x10000200", "Position9"),
    (944031, "0x10000400", "Position10"),
    (944032, "0x10000800", " "),
]


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeBitsReadMsgTxt(
    self,
    tc_no=0,
    statusReasonAux=None,
    expMsgTxt="undef",
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    assert tc_no != 0
    assert statusReasonAux is not None
    assert expMsgTxt != "undef"

    maxTime = 2  # 2 seconds maximum to let ripple through
    passed = False
    self.axisMr.setValueOnSimulator(tc_no, "nStatReasAUX", statusReasonAux)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)

    while maxTime > 0:
        actMsgTxt = self.axisCom.get("-MsgTxt")
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} maxTime={maxTime:.2f} expMsgTxt='{expMsgTxt!r}' actMsgTxt={actMsgTxt!r} "
        )

        if actMsgTxt == expMsgTxt:
            passed = True
            maxTime = 0
        else:
            time.sleep(polltime)
            maxTime = maxTime - polltime

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} tc_no={tc_no} passed={passed} statusReasonAux={statusReasonAux} expMsgTxt='{expMsgTxt}' actMsgTxt={actMsgTxt!r}"
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

    def test_TC_944020(self):
        for tc in aux07TCs:
            tc_no = tc[0]
            statusReasonAux = tc[1]
            expMsgTxt = tc[2]
            writeBitsReadMsgTxt(
                self,
                tc_no=tc_no * 10 + 1,
                statusReasonAux=statusReasonAux,
                expMsgTxt=expMsgTxt,
            )

    def xest_TC_94499999(self):
        tc_no = 94499999
        writeBitsReadMsgTxt(
            self,
            tc_no=tc_no,
            statusReasonAux=idxStatusCodeIDLE,
            errorId=0,
            expMsgTxt="",
        )
