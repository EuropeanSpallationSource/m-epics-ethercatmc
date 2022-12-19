#!/usr/bin/env python

import datetime
import inspect
import unittest
import os
import sys
import time
from AxisMr import AxisMr
from AxisCom import AxisCom

filnam = os.path.basename(__file__)[0:3]

###

polltime = 0.2


def lineno():
    return inspect.currentframe().f_back.f_lineno


def startAndPowerOff(self, tc_no, startpos, field_name, field_value):
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}")
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
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
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} bError_1={int(bError_1)} nErrorId_1=0x{int(nErrorId_1):04X}"
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
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} sleep counter = {int(counter)}"
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
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} bError_1={bError_1} msta_1={self.axisMr.getMSTAtext(msta_1)}"
    )
    testPassed = int(bError_1) == 0
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    return testPassed


def powerOffAndStart(self, tc_no, startpos, field_name, field_value):
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no}")
    caughtException = False
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    self.axisCom.put(".CNEN", 1)
    self.axisMr.waitForPowerOn(tc_no, 8.0)
    self.axisMr.moveWait(tc_no, startpos)

    self.axisCom.putDbgStrToLOG("PwrOf " + str(tc_no), wait=True)
    self.axisCom.put("-PwrAuto", 0)
    self.axisCom.put(".CNEN", 0)
    self.axisCom.putDbgStrToLOG("MoveS " + str(tc_no), wait=True)
    self.axisCom.put(field_name, field_value)

    # self.axisMr.waitForStart(tc_no, 3.0)
    # self.axisMr.waitForStop(tc_no, 3.0)
    try:
        self.axisMr.waitForStartAndDone(tc_no, 6.0)
    except:
        caughtException = True
        msta = int(self.axisCom.get(".MSTA", use_monitor=False))
        if msta & self.axisMr.MSTA_BIT_MOVING:
            self.axisCom.putDbgStrToLOG("ESTOP " + str(tc_no), wait=True)
            self.axisCom.put(".STOP", 1)
            self.axisMr.waitForStop(tc_no, 3.0)

    self.axisCom.putDbgStrToLOG("MoveE " + str(tc_no), wait=True)

    msta_1 = int(self.axisCom.get(".MSTA", use_monitor=False))
    bError_1 = self.axisCom.get("-Err", use_monitor=False)
    nErrorId_1 = self.axisCom.get("-ErrId", use_monitor=False)
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} bError_1={int(bError_1)} nErrorId_1=0x{int(nErrorId_1):04X} caughtException={caughtException}"
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
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} sleep counter = {int(counter)}"
            )
            msta = int(self.axisCom.get(".MSTA", use_monitor=False))
            bError = self.axisCom.get("-Err", use_monitor=False)
            nErrorId = self.axisCom.get("-ErrId", use_monitor=False)
            counter = counter - 1
            if counter == 0:
                break

    # Re-home if needed
    self.axisMr.powerOnHomeAxis(tc_no)

    # Restore the values we had before this test started
    self.axisCom.put(".CNEN", self.saved_CNEN)
    if self.saved_CNEN:
        self.axisMr.waitForPowerOn(tc_no + "restore", 8.0)
    else:
        self.axisMr.waitForPowerOff(tc_no + "restore", 3.0)

    self.axisCom.put("-PwrAuto", self.saved_PwrAuto)

    # Run the assert after we have restored the original state
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tc_no} bError_1={bError_1} msta_1={self.axisMr.getMSTAtext(msta_1)}"
    )
    testPassed = not caughtException and (int(bError_1) != 0 or int(nErrorId_1) != 0)
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    return testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    msta = int(axisCom.get(".MSTA"))
    # self.axisCom.put('-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20], wait=True)
    saved_HLM = axisCom.get(".HLM")
    saved_LLM = axisCom.get(".LLM")
    saved_CNEN = axisCom.get(".CNEN")
    saved_PwrAuto = axisCom.get("-PwrAuto")

    # Assert that motor is homed
    def test_TC_2011(self):
        tc_no = "2011"
        self.axisMr.powerOnHomeAxis(tc_no)

    #    # Jog, wait for start, power off, check for no error, reset error if needed
    #    def test_TC_2012(self):
    #        tc_no = "2012"
    #        # startpos = (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
    #        startpos = self.saved_LLM
    #        testPassed = startAndPowerOff(self, tc_no, startpos, ".JOGF", 1)
    #        assert testPassed

    # Move, wait for start, power off, check for no error, reset error if needed
    def test_TC_2013(self):
        tc_no = "2013"
        startpos = self.axisCom.get(".VAL")
        endpos = startpos + 3 * self.axisCom.get(".VELO")
        if endpos > self.saved_HLM:
            startpos = self.saved_LLM
            endpos = self.saved_HLM
        testPassed = startAndPowerOff(self, tc_no, startpos, ".VAL", endpos)
        # .VAL and .RBV are out of sync
        self.axisCom.put(".SYNC", 1)
        assert testPassed

    #    # Home, wait for start, power off, check for no error, reset error if needed
    #    def test_TC_2014(self):
    #        tc_no = "2014"
    #        startpos = (self.saved_HLM + self.saved_LLM) / 2.0
    #        testPassed = startAndPowerOff(self, tc_no, startpos, ".HOMF", 1)
    #        assert testPassed

    ###############################
    # power off, try to jog, check for error, reset error if needed
    def test_TC_2015(self):
        tc_no = "2015"
        # startpos = (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        startpos = self.saved_LLM
        testPassed = powerOffAndStart(self, tc_no, startpos, ".JOGF", 1)
        assert testPassed

    # power off, move, check for error, reset error if needed
    def test_TC_2016(self):
        tc_no = "2016"
        startpos = self.axisCom.get(".VAL")
        endpos = startpos + 3 * self.axisCom.get(".VELO")
        if endpos > self.saved_HLM:
            startpos = self.saved_LLM
            endpos = self.saved_HLM
        testPassed = powerOffAndStart(self, tc_no, startpos, ".VAL", endpos)
        assert testPassed

    # power off, try to move, check for error, reset error if needed
    def test_TC_2017(self):
        tc_no = "2017"
        startpos = (self.saved_HLM + self.saved_LLM) / 2.0
        testPassed = powerOffAndStart(self, tc_no, startpos, ".HOMF", 1)
        assert testPassed

    def teardown_class(self):
        tc_no = int(filnam) * 10000 + 9999
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisCom.close()
