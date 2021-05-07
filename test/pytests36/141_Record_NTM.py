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

filnam = "140xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


def moveVALnewRBVnewValRtryDly(
    self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, rtry, dly
):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    # Go to start
    oldDLY = self.axisCom.get(".DLY")
    oldNTM = self.axisCom.get(".NTM")
    oldRTRY = self.axisCom.get(".RTRY")
    oldSPAM = self.axisCom.get(".SPAM")
    self.axisCom.put(".DLY", dly)
    self.axisCom.put(".NTM", 1)
    self.axisCom.put(".RTRY", rtry)
    self.axisCom.put(".SPAM", 2047)
    velo = self.axisCom.get(".VELO")
    vmax = self.axisCom.get(".VMAX")

    if vmax > 0.0:
        self.axisCom.put(".VELO", vmax)
    self.axisMr.moveWait(tc_no, startpos)

    if vmax > 0.0:
        self.axisCom.put(".VELO", velo)

    # Start a movement
    self.axisCom.put(".VAL", firstVal)
    self.axisMr.waitForStart(tc_no, 2.0)

    maxDelta = velo + 2.0  # around our point
    timeout = self.axisMr.calcTimeOut(pointOfReturnPos + 1.0, velo)
    self.axisMr.waitForValueChanged(tc_no, ".RBV", pointOfReturnPos, maxDelta, timeout)

    timeout = self.axisMr.calcTimeOut(secondVal, velo)
    self.axisCom.put(".VAL", secondVal)

    rdbd = self.axisCom.get(".RDBD")
    testPassed = self.axisMr.waitForValueChanged(
        tc_no, ".RBV", secondVal, rdbd, timeout
    )
    self.axisMr.waitForStop(tc_no, timeout + dly + 1.0)
    testPassed = testPassed and self.axisMr.postMoveCheck(tc_no)

    self.axisCom.put(".DLY", oldDLY)
    self.axisCom.put(".NTM", oldNTM)
    self.axisCom.put(".RTRY", oldRTRY)
    self.axisCom.put(".SPAM", oldSPAM)
    if testPassed:
        self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
    else:
        self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
    assert testPassed


def moveVALnewRBVnewVal(self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal):
    rtry = 0
    moveVALnewRBVnewValRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, rtry, 0.0
    )
    tc_no = int(tc_no) + 1
    moveVALnewRBVnewValRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, rtry, 0.5
    )
    tc_no = int(tc_no) + 1
    rtry = 1
    moveVALnewRBVnewValRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, rtry, 0.0
    )
    tc_no = int(tc_no) + 1
    moveVALnewRBVnewValRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, rtry, 0.5
    )


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam} url_string={url_string}"
    )

    axisCom = AxisCom(url_string, log_debug=False)
    axisMr = AxisMr(axisCom)
    motorPvName = axisCom.getMotorPvName()
    hlm = axisCom.get(".HLM")
    llm = axisCom.get(".LLM")
    velo = axisCom.get(".VELO")
    vmax = axisCom.get(".VMAX")
    msta = int(axisCom.get(".MSTA"))

    def test_TC_1400(self):
        tc_no = "1400"
        if not (self.msta & self.axisMr.MSTA_BIT_HOMED):
            self.axisMr.powerOnHomeAxis(tc_no)
            self.msta = int(self.axisCom.get(".MSTA"))
            self.assertNotEqual(
                0,
                self.msta & self.axisMr.MSTA_BIT_HOMED,
                "MSTA.homed (Axis is not homed)",
            )

    def test_TC_14010(self):
        tc_no = "14010"
        llm = self.llm
        hlm = self.hlm
        startpos = llm
        firstVal = hlm
        pointOfReturnPos = (llm + hlm) / 2
        secondVal = (3 * llm + 1 * hlm) / 4
        moveVALnewRBVnewVal(
            self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal
        )
