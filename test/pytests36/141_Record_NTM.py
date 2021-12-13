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

filnam = "141xx.py"
###


def lineno():
    return inspect.currentframe().f_back.f_lineno


# Calculate a (small) range to let us travel max 6 seconds
# For "fast" motors this may be the whole distance between LLM and HLM
# for "slow" motors this is less to travel
def calcDistanceMax4Seconds(llm, hlm, velo, fourSeconds):
    distanceMax6Seconds = fourSeconds * velo
    travelRange = hlm - llm
    if travelRange > 0.0 and travelRange > distanceMax6Seconds:
        travelRange = distanceMax6Seconds
    #    print(
    #        f"{filnam} calcDistanceMax4Seconds llm={llm} hlm={hlm} travelRange={travelRange}"
    #    )

    return travelRange


def moveVALnewRBVnewValNtmRtryDly(
    self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm, rtry, dly
):
    self.axisCom.putDbgStrToLOG("Start " + str(tc_no), wait=True)
    # Go to start
    oldDLY = self.axisCom.get(".DLY")
    oldNTM = self.axisCom.get(".NTM")
    oldRTRY = self.axisCom.get(".RTRY")
    oldSPAM = self.axisCom.get(".SPAM")
    self.axisCom.put(".DLY", dly)
    self.axisCom.put(".NTM", ntm)
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

    # Extra long timeout: The motor may overshoot, kind of
    timeout = 2 * self.axisMr.calcTimeOut(secondVal, velo)
    self.axisCom.putDbgStrToLOG("NewVAL " + str(tc_no), wait=True)
    self.axisCom.put(".VAL", secondVal)

    rdbd = self.axisCom.get(".RDBD")
    valueChangedOK = self.axisMr.waitForValueChanged(
        tc_no, ".RBV", secondVal, rdbd, timeout
    )
    self.axisMr.waitForStop(tc_no, timeout + dly + 1.0)
    postMoveCheckOK = self.axisMr.postMoveCheck(tc_no)
    testPassed = valueChangedOK and postMoveCheckOK
    print(
        f"{tc_no} moveVALnewRBVnewValNtmRtryDly valueChangedOK={valueChangedOK} postMoveCheckOK={postMoveCheckOK}"
    )

    self.axisCom.put(".DLY", oldDLY)
    self.axisCom.put(".NTM", oldNTM)
    self.axisCom.put(".RTRY", oldRTRY)
    self.axisCom.put(".SPAM", oldSPAM)
    if testPassed:
        self.axisCom.putDbgStrToLOG("Passed " + str(tc_no), wait=True)
    else:
        self.axisCom.putDbgStrToLOG("Failed " + str(tc_no), wait=True)
    assert testPassed


def moveVALnewRBVnewValNtm(
    self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm
):
    rtry = 0
    moveVALnewRBVnewValNtmRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm, rtry, 0.0
    )
    tc_no = int(tc_no) + 1
    moveVALnewRBVnewValNtmRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm, rtry, 0.5
    )
    tc_no = int(tc_no) + 1
    rtry = 1
    moveVALnewRBVnewValNtmRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm, rtry, 0.0
    )
    tc_no = int(tc_no) + 1
    moveVALnewRBVnewValNtmRtryDly(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm, rtry, 0.5
    )


def moveVALnewRBVnewValWrapper(
    self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal
):

    ntm = 0
    moveVALnewRBVnewValNtm(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm
    )
    tc_no = int(tc_no) + 10
    ntm = 1
    moveVALnewRBVnewValNtm(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm
    )

    vers = float(self.axisCom.get(".VERS"))
    if vers < 7.07 or vers > 7.09:
        return
    mflg = int(self.axisCom.get(".MFLG"))
    mf_ntm_update_bit = 32
    if not mflg & mf_ntm_update_bit:
        return
    ntm = 2
    tc_no = int(tc_no) + 10
    moveVALnewRBVnewValNtm(
        self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal, ntm
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
    travelDistance = calcDistanceMax4Seconds(llm, hlm, velo, 4.0)

    def test_TC_14100(self):
        tc_no = "14100"
        self.axisMr.powerOnHomeAxis(tc_no)

    #
    # NTM starting from LLm going forward
    #
    def test_TC_141100(self):
        tc_no = "141100"
        startpos = self.llm
        firstVal = startpos + self.travelDistance
        pointOfReturnPos = startpos + self.travelDistance * 0.5
        secondVal = startpos + self.travelDistance * 0.75
        moveVALnewRBVnewValWrapper(
            self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal
        )

    #
    # NTM starting from LLM goiing forward, and then again to LLM
    #
    def test_TC_141200(self):
        tc_no = "141200"
        startpos = self.llm
        firstVal = startpos + self.travelDistance
        pointOfReturnPos = startpos + self.travelDistance * 0.5
        moveVALnewRBVnewValWrapper(
            self, tc_no, startpos, firstVal, pointOfReturnPos, startpos
        )

    #
    # NTM starting from HLM going backward
    #
    def test_TC_141300(self):
        tc_no = "141300"
        startpos = self.hlm
        firstVal = startpos - self.travelDistance
        pointOfReturnPos = startpos - self.travelDistance * 0.5
        secondVal = startpos - self.travelDistance * 0.75
        moveVALnewRBVnewValWrapper(
            self, tc_no, startpos, firstVal, pointOfReturnPos, secondVal
        )

    #
    # NTM starting from HLM going backward, and then again to HLM
    #
    def test_TC_141400(self):
        tc_no = "141400"
        startpos = self.hlm
        firstVal = startpos - self.travelDistance
        pointOfReturnPos = startpos - self.travelDistance * 0.5

        moveVALnewRBVnewValWrapper(
            self, tc_no, startpos, firstVal, pointOfReturnPos, startpos
        )
