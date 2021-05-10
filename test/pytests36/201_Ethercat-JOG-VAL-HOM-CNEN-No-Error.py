#!/usr/bin/env python

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

filnam = "201xx.py"

###

polltime = 0.2


def startAndPowerOff(self, tc_no, startpos, field_name, field_value):
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    self.axisCom.put(".CNEN", 1)
    self.axisMr.waitForPowerOn(tc_no, 8.0)
    self.axisMr.moveWait(tc_no, startpos)

    self.axisCom.put("-PwrAuto", 0)
    self.axisCom.put(".CNEN", 1)
    self.axisMr.waitForPowerOn(tc_no, 8.0)
    self.axisCom.put(field_name, field_value)

    self.axisMr.waitForStart(tc_no, 3.0)
    self.axisCom.put(".CNEN", 0)
    self.axisMr.waitForStop(tc_no, 3.0)

    msta_1 = int(self.axisCom.get(".MSTA", use_monitor=False))
    bError_1 = self.axisCom.get("-Err", use_monitor=False)
    nErrorId_1 = self.axisCom.get("-ErrId", use_monitor=False)
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} bError_1={int(bError_1)} nErrorId_1=0x{int(nErrorId_1):04X}"
    )

    if bError_1 != 0:
        self.axisMr.resetAxis(tc_no)
        # Loop until moving has stopped and error has been reseted
        counter = 7
        msta = msta_1
        nErrorId = nErrorId_1
        bError = bError_1
        while msta & self.axisMr.MSTA_BIT_MOVING or bError != 0 or nErrorId != 0:
            time.sleep(polltime)
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} sleep counter = {int(counter)}"
            )
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            bError = self.axisCom.get("-Err", use_monitor=False)
            nErrorId = self.axisCom.get("-ErrId", use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break

    # Restore the values we had before this test started
    self.axisCom.put(".CNEN", self.saved_CNEN)
    if self.saved_CNEN:
        self.axisMr.waitForPowerOn(tc_no + "restore", 8.0)
    else:
        self.axisMr.waitForPowerOff(tc_no + "restore", 3.0)

    self.axisCom.put("-PwrAuto", self.saved_PwrAuto)

    # Run the assert after we have restored the original state
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} bError_1={bError_1} msta_1={self.axisMr.getMSTAtext(msta_1)}"
    )
    testPassed = int(bError_1) == 0
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom)

    msta = int(axisCom.get(".MSTA"))
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    saved_HLM = axisCom.get(".HLM")
    saved_LLM = axisCom.get(".LLM")
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")

    # Assert that motor is homed
    def test_TC_2011(self):
        tc_no = "2011"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # Jog, wait for start, power off, check for no error, reset error if needed
    def test_TC_2012(self):
        tc_no = "2012"
        # startpos = (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        startpos = self.saved_LLM
        startAndPowerOff(self, tc_no, startpos, ".JOGF", 1)

    # Move, wait for start, power off, check for no error, reset error if needed
    def test_TC_2013(self):
        tc_no = "2013"
        startpos = self.axisCom.get(".VAL")
        endpos = startpos + 3 * self.axisCom.get(".VELO")
        if endpos > self.saved_HLM:
            startpos = self.saved_LLM
            endpos = self.saved_HLM
        startAndPowerOff(self, tc_no, startpos, ".VAL", endpos)

    # Move, wait for start, power off, check for no error, reset error if needed
    def test_TC_2013(self):
        tc_no = "2013"
        startpos = (self.saved_HLM + self.saved_LLM) / 2.0
        startAndPowerOff(self, tc_no, startpos, ".HOMF", 1)
