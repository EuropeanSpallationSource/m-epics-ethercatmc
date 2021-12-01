#!/usr/bin/env python
#

import datetime
import unittest
import os
import sys
from AxisMr import AxisMr
from AxisCom import AxisCom

import time
import math
import inspect

filnam = "910xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


# How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1

noFRAC = 1.0
withFRAC = 1.5


def motorInitTC(self, tc_no, frac, encRel):
    self.axisCom.put(".FRAC", frac)
    self.axisCom.put(".UEIP", encRel)
    self.axisCom.put(".RTRY", 1)
    msta = int(self.axisCom.get(".MSTA", use_monitor=False))
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} {tc_no}:{int(lineno())} motorInitTC msta={self.axisMr.getMSTAtext(msta)}"
    )


def setMotorStartPos(self, tc_no, startpos):
    self.axisMr.setValueOnSimulator(tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    self.axisMr.doSTUPandSYNC(tc_no)


def jogAndBacklash(self, tc_no, frac, encRel, StartPos, EndPos, myJOGX):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInitTC(self, tc_no, frac, encRel)
    setMotorStartPos(self, tc_no, StartPos)
    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    if myJOGX == "JOGF":
        myDirection = 1
    elif myJOGX == "JOGR":
        myDirection = 0
    else:
        assert 0
    self.axisMr.writeExpFileJOG_BDST(
        tc_no,
        expFileName,
        myDirection,
        frac,
        encRel,
        StartPos,
        EndPos,
    )
    field_name = "." + myJOGX
    # Add the dot between the motorRecord name and the field
    self.axisCom.put(field_name, 1)
    time.sleep(1)
    self.axisMr.setValueOnSimulator(tc_no, "fActPosition", EndPos)
    self.axisCom.put(field_name, 0)
    time_to_wait = 500
    self.axisMr.waitForMipZero(tc_no, time_to_wait)
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")

    testPassed = self.axisMr.cmpUnlinkExpectedActualFile(
        tc_no, expFileName, actFileName
    )
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no), wait=True)
    assert testPassed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])

    myPOSlow = axisMr.myPOSlow
    myPOSmid = axisMr.myPOSmid
    myPOShig = axisMr.myPOShig

    def test_TC_91000(self):
        tc_no = "91000"
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no), wait=True)
        self.axisMr.initializeMotorRecordSimulatorAxis(tc_no)
        self.axisMr.motorInitAllForBDST(tc_no)
        self.axisCom.put(".SPAM", 255)
        self.axisCom.put("-DbgStrToLOG", "Finish " + str(tc_no), wait=True)

    # JOG forward & backlash compensation, absolute
    def test_TC_91011(self):
        jogAndBacklash(
            self,
            91011,
            noFRAC,
            use_abs,
            self.myPOSlow,
            self.myPOSmid,
            "JOGF",
        )

    # JOG forward & backlash compensation, relative
    def test_TC_91012(self):
        jogAndBacklash(
            self,
            91012,
            noFRAC,
            use_rel,
            self.myPOSmid,
            self.myPOSlow,
            "JOGF",
        )

    # JOG backward & backlash compensation, absolute
    def test_TC_91021(self):
        jogAndBacklash(
            self,
            91021,
            noFRAC,
            use_abs,
            self.myPOSlow,
            self.myPOSmid,
            "JOGR",
        )

    # JOG backward & backlash compensation, relative
    def test_TC_91022(self):
        jogAndBacklash(
            self,
            91022,
            noFRAC,
            use_rel,
            self.myPOSmid,
            self.myPOSlow,
            "JOGR",
        )

    # JOG forward & backlash compensation, absolute
    def test_TC_91031(self):
        jogAndBacklash(
            self,
            91031,
            withFRAC,
            use_abs,
            self.myPOSlow,
            self.myPOSmid,
            "JOGF",
        )

    # JOG forward & backlash compensation, relative
    def test_TC_91032(self):
        jogAndBacklash(
            self,
            91032,
            withFRAC,
            use_rel,
            self.myPOSmid,
            self.myPOSlow,
            "JOGF",
        )

    # JOG backward & backlash compensation, absolute
    def test_TC_91041(self):
        jogAndBacklash(
            self,
            91041,
            withFRAC,
            use_abs,
            self.myPOSlow,
            self.myPOSmid,
            "JOGR",
        )

    # JOG backward & backlash compensation, relative
    def test_TC_91042(self):
        jogAndBacklash(
            self,
            91042,
            withFRAC,
            use_rel,
            self.myPOSmid,
            self.myPOSlow,
            "JOGR",
        )
