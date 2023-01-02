#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import datetime
import inspect
import math
import os
import sys
import time
import unittest

from AxisTestUtil import AxisTestUtil
import AxisComm as AxisComm

filnam = os.path.basename(__file__)[0:3]
polltime = 0.1


def lineno():
    return inspect.currentframe().f_back.f_lineno


def checkForEmergenyStop(self, tc_no, wait, direction, oldRBV, rdbd):
    fName = "checkForEmergenyStop"
    outOfRange = 0
    lls = 0
    hls = 0
    outOfRange = 0
    rbv = self.axisComm.getActPos(tc_no=tc_no)
    dmov = self.axisComm.getDoneStatus(tc_no=tc_no)
    movn = self.axisComm.getBusyStatus(tc_no=tc_no)
    if (rbv - oldRBV) > 2 * rdbd:
        outOfRange = 1
    if (oldRBV - rbv) > 2 * rdbd:
        outOfRange = -1
    hls = self.axisComm.getLimitFwd()
    lls = self.axisComm.getLimitBwd()
    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} wait={wait:.2f} dmov={dmov} movn={movn} direction={direction} hls={hls} lls={lls} outOfRange={outOfRange} oldRBV={oldRBV} rbv={rbv}"
    )

    if (hls and (direction > 0)) or (lls and (direction <= 0)) or outOfRange != 0:
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} haltAndDisable"
        )
        self.axisComm.haltAxis()
        self.axisComm.disableAxis()

    return (lls, hls, movn, dmov, outOfRange)


def waitForStart(self, tc_no, wait_for, direction, oldRBV):
    fName = "waitForStart"
    rdbd = self.axisComm.getAxisTargetPositionWindow()
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, tc_no + " waitForStart", wait_for, direction, oldRBV, rdbd
        )
        rbv = self.axisComm.getActPos(tc_no=tc_no)
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False


def waitForStop(self, tc_no, wait_for, direction, oldRBV, rdbd):
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self,
            tc_no + " waitForStop",
            wait_for,
            direction,
            oldRBV,
            rdbd,
        )

        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False


def tweakToLimit(self, tc_no, direction):
    fName = "tweakToLimit"
    self.axisComm.putDbgStrToLOG("Start " + str(tc_no) + " dir=" + str(direction))
    assert direction
    old_DHLM = self.axisComm.getSoftLimitFwdValue()
    old_DLLM = self.axisComm.getSoftLimitBwdValue()

    # switch off the soft limits, save the values
    self.axisComm.setSoftLimitsOff(tc_no=tc_no)

    # If we reached the limit switch, we are fine and
    # can reset the error
    self.axisComm.resetAxis(tc_no=tc_no)
    self.axisComm.enableAxis(tc_no=tc_no)
    # Step through the range in 20 steps or so
    deltaToMove = (old_DHLM - old_DLLM) / 20
    maxDeltaAfterMove = deltaToMove / 2
    # soft limit range + 110 % (some test crates simulate a slit, with restrictive soft limits
    maxTweaks = (old_DHLM - old_DLLM) * 2.1 / deltaToMove
    assert maxTweaks > 0
    stopTheLoop = False
    count = 0
    lls = 0
    hls = 0
    while not stopTheLoop:
        oldRBV = self.axisComm.getActPos(tc_no=tc_no)
        if direction > 0:
            destination = oldRBV + deltaToMove
        else:
            destination = oldRBV - deltaToMove
            print(
                f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} direction={direction} oldRBV={oldRBV} destination={destination}"
            )

        self.axisComm.moveAbsolute(destination)

        ret1 = waitForStart(self, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, tc_no, 10.0, direction, oldRBV, deltaToMove)
        hls = self.axisComm.getLimitFwd()
        lls = self.axisComm.getLimitBwd()

        rbv = self.axisComm.getActPos(tc_no=tc_no)
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} ret1={ret1} ret2={ret2} rbv={rbv} hls={hls} lls={lls}"
        )
        self.axisComm.haltAxis()
        count += 1
        if count > maxTweaks:
            stopTheLoop = True
        if (hls and (direction > 0)) or (lls and (direction <= 0)):
            stopTheLoop = True
        elif not lls and not hls:
            inPosition = self.axisComm.calcAlmostEqual(
                destination, rbv, maxDeltaAfterMove, tc_no=tc_no
            )
            if not inPosition:
                stopTheLoop = True
                self.axisComm.haltAxis()
                self.axisComm.disableAxis()
        else:
            inPosition = -1

        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} old_DHLM={old_DHLM}  old_DLLM={old_DLLM} deltaToMove={deltaToMove} count={count} maxTweaks={maxTweaks} stopTheLoop={stopTheLoop}"
        )

    print(
        f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()}/{fName} {tc_no} direction={direction} hls={hls} lls={lls}"
    )
    # Put back the limits for the next run
    self.axisComm.setSoftLimitsOn(tc_no=tc_no, low_limit=old_DLLM, high_limit=old_DHLM)

    passed = True
    # Check if we reached the limit switch, prepare to move away from it
    if direction > 0:
        if not hls:
            passed = False
        newPos = rbv - deltaToMove
    else:
        if not lls:
            passed = False
        newPos = rbv + deltaToMove

    # If we reached the limit switch, we are fine and
    # can reset the error
    self.axisComm.resetAxis(tc_no=tc_no)
    if passed:
        self.axisComm.putDbgStrToLOG("Passed " + str(tc_no))
    else:
        self.axisComm.putDbgStrToLOG("Failed " + str(tc_no))
    assert passed


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisTestUtil = AxisTestUtil(url_string, log_debug=True)
    axisComm = axisTestUtil.getAxis()

    old_Enable = axisComm.getEnabledStatus()

    def test_TC_09002(self):
        tc_no = "09002"
        print(f"{tc_no}")
        direction = 1
        if self.axisComm.getLimitFwd():
            direction = -1
        tweakToLimit(self, tc_no, direction)

    def test_TC_09003(self):
        tc_no = "09003"
        print(f"{tc_no}")
        direction = -1
        if self.axisComm.getLimitBwd():
            direction = 1
        tweakToLimit(self, tc_no, direction)

    def test_TC_09004(self):
        tc_no = "09004"
        print(f"{tc_no}")
        self.axisComm.resetAxis(tc_no=tc_no)
        if self.old_Enable:
            self.axisComm.enableAxis()

    def teardown_class(self):

        tc_no = "090999"
        print(
            f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {filnam}:{lineno()} {tc_no} teardown_class"
        )
        self.axisTestUtil.close()
