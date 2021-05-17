#!/usr/bin/env python

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time

filnam = "203xx.py"

###

polltime = 0.2


def startAndSoftLimitsOff(self, tc_no, startpos, field_name, field_value):
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}")
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    self.axisMr.moveWait(tc_no, startpos)

    self.axisCom.put(field_name, field_value)
    self.axisMr.waitForStart(tc_no, 3.0)
    self.axisMr.setSoftLimitsOff(tc_no)

    # Calculate the timeout, based on the driving range
    range_postion = self.axisCom.get(".HLM") - self.axisCom.get(".LLM")
    hvel = self.axisCom.get(".HVEL")
    accl = self.axisCom.get(".ACCL")
    if range_postion > 0 and hvel > 0:
        time_to_wait = 1 + 2 * range_postion / hvel + 2 * accl
    else:
        time_to_wait = 180

    self.axisMr.waitForStop(tc_no, time_to_wait)

    msta_1 = int(self.axisCom.get(".MSTA", use_monitor=False))
    bError_1 = self.axisCom.get("-Err", use_monitor=False)
    nErrorId_1 = self.axisCom.get("-ErrId", use_monitor=False)
    if msta_1 & self.axisMr.MSTA_BIT_HOMED:
        homed = 1
    else:
        homed = 0
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} homed={homed} bError_1={int(bError_1)} nErrorId_1=0x{int(nErrorId_1):04X}"
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

    # Run the assert after we have restored the original state
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no} bError_1={bError_1} msta_1={self.axisMr.getMSTAtext(msta_1)}"
    )
    testPassed = int(bError_1) == 0 and homed == 1
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    return testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    msta = int(axisCom.get(".MSTA"))
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    saved_HLM = axisCom.get(".HLM")
    saved_LLM = axisCom.get(".LLM")

    # Assert that motor is homed
    def test_TC_2031(self):
        tc_no = "2031"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    # Home, wait for start, soft limits off, check for no error, reset error if needed
    def test_TC_2031(self):
        tc_no = "2032"
        startpos = (self.saved_HLM + self.saved_LLM) / 2.0
        testPassed = startAndSoftLimitsOff(self, tc_no, startpos, ".HOMF", 1)
        assert testPassed

    # high limit switch
    def test_TC_2032(self):
        tc_no = "2032"
        direction = 1
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)))
            passed = self.axisMr.moveIntoLS(
                tc_no=tc_no,
                direction=direction,
                doSetSoftLimitsOff=False,
                doSetSoftLimitsOn=False,
                paramWhileMove=True,
            )
            if passed:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
            else:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
            assert passed

    # low limit switch
    def test_TC_2033(self):
        tc_no = "2033"
        direction = 0
        msta = int(self.axisCom.get(".MSTA"))
        if msta & self.axisMr.MSTA_BIT_HOMED:
            self.axisCom.put("-DbgStrToLOG", "Start " + str(int(tc_no)))
            passed = self.axisMr.moveIntoLS(
                tc_no=tc_no,
                direction=direction,
                doSetSoftLimitsOff=False,
                doSetSoftLimitsOn=True,
                paramWhileMove=True,
            )
            if passed:
                self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
            else:
                self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
            assert passed
