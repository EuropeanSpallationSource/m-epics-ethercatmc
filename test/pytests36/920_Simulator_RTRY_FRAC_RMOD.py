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

filnam = "920xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


###

myFRAC = 1.5

motorRMOD_D = 0  # "Default"
motorRMOD_A = 1  # "Arithmetic"
motorRMOD_G = 2  # "Geometric"
motorRMOD_I = 3  # "In-Position"

#
# How we move: Absolute (without encoder) or relative (with encode via UEIP)
use_abs = 0
use_rel = 1
# Note: motorRMOD_I is always absolute !


def motorInitTC(self, tc_no, rmod, encRel):
    self.axisCom.put(".RMOD", rmod)
    self.axisCom.put(".UEIP", encRel)


def setMotorStartPos(self, tc_no, startpos):
    self.axisMr.setValueOnSimulator(tc_no, "fActPosition", startpos)
    # Run a status update and a sync
    # Run a status update and a sync
    self.axisMr.doSTUPandSYNC(tc_no)


def positionAndBacklash(self, tc_no, rmod, encRel, motorStartPos, motorEndPos):
    ###########
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))

    mot = self.axisCom.getMotorPvName()
    fileName = "/tmp/" + mot.replace(":", "-") + "-" + str(tc_no)
    expFileName = fileName + ".exp"
    actFileName = fileName + ".act"

    motorInitTC(self, tc_no, rmod, encRel)
    setMotorStartPos(self, tc_no, motorStartPos)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 1)
    # time.sleep(2)
    self.axisMr.setValueOnSimulator(tc_no, "log", actFileName)
    # time.sleep(2)
    #
    self.axisCom.put(".VAL", motorEndPos, wait=True)

    # Create a "expected" file
    expFile = open(expFileName, "w")
    # Positioning
    # 2 different ways to move:
    # - Within the backlash distance and into the backlash direction:
    #   single move with back lash parameters
    # - against the backlash direction -or- bigger than the backlash distance:
    #   two moves, first with moving, second with backlash parameters

    cnt = 1 + int(self.axisCom.get(".RTRY"))
    self.axisMr.writeExpFileRMOD_X(
        tc_no,
        rmod,
        expFile,
        cnt,
        myFRAC,
        encRel,
        motorStartPos,
        motorEndPos,
    )

    expFile.close()
    # self.axisMr.setValueOnSimulator( tc_no, "dbgCloseLogFile", "1")

    time_to_wait = 100
    self.axisMr.waitForStop(tc_no, time_to_wait)
    self.axisMr.setValueOnSimulator(tc_no, "dbgCloseLogFile", "1")
    # time.sleep(2)
    self.axisMr.setValueOnSimulator(tc_no, "bManualSimulatorMode", 0)
    testPassed = self.axisMr.cmpUnlinkExpectedActualFile(
        tc_no, expFileName, actFileName
    )
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

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    msta = int(axisCom.get(".MSTA"))

    axisCom.put("-DbgStrToLOG", "Start " + os.path.basename(__file__)[0:20])
    myPOSlow = axisMr.myPOSlow
    myPOSmid = axisMr.myPOSmid
    myPOShig = axisMr.myPOShig

    def test_TC_92000(self):
        self.axisMr.motorInitAllForBDST(92000)
        self.axisCom.put(".FRAC", myFRAC)

    # motorRMOD_D = 0 # "Default"
    # position forward & backlash compensation, absolute
    def test_TC_92001(self):
        positionAndBacklash(
            self, 92001, motorRMOD_D, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92002(self):
        positionAndBacklash(
            self, 92002, motorRMOD_D, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92003(self):
        positionAndBacklash(
            self, 92003, motorRMOD_D, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92004(self):
        positionAndBacklash(
            self, 92004, motorRMOD_D, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92005(self):
        positionAndBacklash(
            self, 92005, motorRMOD_D, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92006(self):
        positionAndBacklash(
            self, 92006, motorRMOD_D, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92007(self):
        positionAndBacklash(
            self, 92007, motorRMOD_D, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92008(self):
        positionAndBacklash(
            self, 92008, motorRMOD_D, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_A
    # position forward & backlash compensation, absolute
    def test_TC_92011(self):
        positionAndBacklash(
            self, 92011, motorRMOD_A, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92012(self):
        positionAndBacklash(
            self, 92012, motorRMOD_A, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92013(self):
        positionAndBacklash(
            self, 92013, motorRMOD_A, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92014(self):
        positionAndBacklash(
            self, 92014, motorRMOD_A, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92015(self):
        positionAndBacklash(
            self, 92015, motorRMOD_A, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92016(self):
        positionAndBacklash(
            self, 92016, motorRMOD_A, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92017(self):
        positionAndBacklash(
            self, 92017, motorRMOD_A, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92018(self):
        positionAndBacklash(
            self, 92018, motorRMOD_A, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_G
    # position forward & backlash compensation, absolute
    def test_TC_92021(self):
        positionAndBacklash(
            self, 92021, motorRMOD_G, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92022(self):
        positionAndBacklash(
            self, 92022, motorRMOD_G, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92023(self):
        positionAndBacklash(
            self, 92023, motorRMOD_G, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92024(self):
        positionAndBacklash(
            self, 92024, motorRMOD_G, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92025(self):
        positionAndBacklash(
            self, 92025, motorRMOD_G, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92026(self):
        positionAndBacklash(
            self, 92026, motorRMOD_G, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92027(self):
        positionAndBacklash(
            self, 92027, motorRMOD_G, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92028(self):
        positionAndBacklash(
            self, 92028, motorRMOD_G, use_rel, self.myPOSlow, self.myPOSmid
        )

    ###############################################################################
    # motorRMOD_I
    # position forward & backlash compensation, absolute
    def test_TC_92031(self):
        positionAndBacklash(
            self, 92031, motorRMOD_I, use_abs, self.myPOSlow, self.myPOShig
        )

    # position forward & backlash compensation, relative
    def test_TC_92032(self):
        positionAndBacklash(
            self, 92032, motorRMOD_I, use_rel, self.myPOSlow, self.myPOShig
        )

    # position backward & backlash compensation, absolute
    def test_TC_92033(self):
        positionAndBacklash(
            self, 92033, motorRMOD_I, use_abs, self.myPOShig, self.myPOSlow
        )

    # position backward & backlash compensation, relative
    def test_TC_92034(self):
        positionAndBacklash(
            self, 92034, motorRMOD_I, use_rel, self.myPOShig, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92035(self):
        positionAndBacklash(
            self, 92035, motorRMOD_I, use_abs, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, relative
    def test_TC_92036(self):
        positionAndBacklash(
            self, 92036, motorRMOD_I, use_rel, self.myPOSmid, self.myPOSlow
        )

    # position forward inside backlash range, absolute
    def test_TC_92037(self):
        positionAndBacklash(
            self, 92037, motorRMOD_I, use_abs, self.myPOSlow, self.myPOSmid
        )

    # position forward inside backlash range, relative
    def test_TC_92038(self):
        positionAndBacklash(
            self, 92038, motorRMOD_I, use_rel, self.myPOSlow, self.myPOSmid
        )
