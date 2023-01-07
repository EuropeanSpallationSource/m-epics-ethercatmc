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


# counter send into the TestDiscreteOutput record
# Once that TestDiscreteInputput has the same value,
# we know that the whole chain IOC -> MCU -> has been
# processed


def lineno():
    return inspect.currentframe().f_back.f_lineno


def writeReadDiscreteOutput(self, tc_no, val):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.counter = self.counter + 1
    value = val + self.counter
    self.axisCom.put("-TestDiscreteOutput", value)
    timeout = 2.1
    # EPICS uses longin is a signed int32
    passed = self.axisMr.waitForValueChangedInt32(
        tc_no, "-TestDiscreteInput", value, timeout
    )
    pils_stat = val & 0xF0000000
    expSeverity = alarmSeverityValuesInt.get(pils_stat)
    if expSeverity != None:
        actSeverity = int(self.axisCom.get("-TestDiscreteInput.SEVR"))
        if actSeverity != expSeverity:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} tc_no={tc_no} pils_stat={pils_stat:X} actSeverity={actSeverity} expSeverity={expSeverity}"
            )
            passed = False
    expAlarmState = alarmStateValuesInt.get(pils_stat)
    if expAlarmState != None:
        actAlarmState = int(self.axisCom.get("-TestDiscreteInput.STAT"))
        if actAlarmState != expAlarmState:
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} tc_no={tc_no} pils_stat={pils_stat:X} actAlarmState={actAlarmState} expAlarmState={expAlarmState}"
            )
            passed = False
    if passed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert passed


def writeReadDiscreteOutput2(self, tc_no, val):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.counter = self.counter + 1
    value = val + self.counter
    self.axisCom.put("-TestDiscreteOutput", value)
    timeout = 2.1
    # EPICS uses longin is a signed int32
    passed = self.axisMr.waitForValueChangedInt32(
        tc_no, "-TestDiscreteInput", value, timeout
    )
    (expSeverity, expAlarmState) = alarmStatSevrValuesInt[val]
    actSeverity = int(self.axisCom.get("-TestDiscreteInput.SEVR"))
    if actSeverity != expSeverity:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} tc_no={tc_no} val={val:X} actSeverity={actSeverity} expSeverity={expSeverity}"
        )
        passed = False
    actAlarmState = int(self.axisCom.get("-TestDiscreteInput.STAT"))
    if actAlarmState != expAlarmState:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} tc_no={tc_no} pils_stat={pils_stat:X} actAlarmState={actAlarmState} expAlarmState={expAlarmState}"
        )
        passed = False
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

    counter = 1
    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    def test_TC_94000(self):
        tc_no = 94002
        val = idxStatusCodeRESET
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94001(self):
        tc_no = 94001
        val = idxStatusCodeIDLE
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94002(self):
        tc_no = 94002
        val = idxStatusCodePOWEROFF
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94003(self):
        tc_no = 94003
        val = idxStatusCodeWARN
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94004(self):
        tc_no = 94004
        val = idxStatusCodeERR4
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94005(self):
        tc_no = 94005
        val = idxStatusCodeSTART
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94006(self):
        tc_no = 94006
        val = idxStatusCodeBUSY
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94007(self):
        tc_no = 94007
        val = idxStatusCodeSTOP
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94008(self):
        tc_no = 94008
        val = idxStatusCodeERROR
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94009(self):
        tc_no = 94009
        val = idxStatusCodeERR9
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94010(self):
        tc_no = 94010
        val = idxStatusCodeERR10
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94011(self):
        val = idxStatusCodeERR11
        tc_no = 94011
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94012(self):
        val = idxStatusCodeERR12
        tc_no = 94012
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94013(self):
        tc_no = 94013
        val = idxStatusCodeERR13
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94014(self):
        tc_no = 94014
        val = idxStatusCodeERR14
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_94015(self):
        tc_no = 94015
        val = idxStatusCodeERR15
        writeReadDiscreteOutput(self, tc_no, val)

    def test_TC_940100(self):
        tc_no = 940100
        val = idxStatusCodeWARN + idxReasonBitHigh
        writeReadDiscreteOutput2(self, tc_no, val)

    def test_TC_940101(self):
        tc_no = 940101
        val = idxStatusCodeERROR + idxReasonBitHigh
        writeReadDiscreteOutput2(self, tc_no, val)

    def test_TC_940102(self):
        tc_no = 940102
        val = idxStatusCodeWARN + idxReasonBitLow
        writeReadDiscreteOutput2(self, tc_no, val)

    def test_TC_940103(self):
        tc_no = 940103
        val = idxStatusCodeERROR + idxReasonBitLow
        writeReadDiscreteOutput2(self, tc_no, val)
