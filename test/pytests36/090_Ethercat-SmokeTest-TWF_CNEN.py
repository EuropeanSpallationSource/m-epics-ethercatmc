#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import unittest
import os
import sys
import math
import time

from AxisMr import AxisMr
from AxisCom import AxisCom

import inspect

###

polltime = 0.1


def lineno():
    return inspect.currentframe().f_back.f_lineno


def checkForEmergenyStop(self, tc_no, wait, direction, oldRBV, TweakValue):
    outOfRange = 0
    lls = 0
    hls = 0
    outOfRange = 0
    rbv = self.axisCom.get(".RBV", use_monitor=False)
    msta = int(self.axisCom.get(".MSTA", use_monitor=False))
    dmov = int(self.axisCom.get(".DMOV", use_monitor=False))
    movn = int(self.axisCom.get(".MOVN", use_monitor=False))
    if (rbv - oldRBV) > 2 * TweakValue:
        outOfRange = 1
    if (oldRBV - rbv) > 2 * TweakValue:
        outOfRange = -1
    if msta & self.axisMr.MSTA_BIT_MINUS_LS:
        lls = 1
    if msta & self.axisMr.MSTA_BIT_PLUS_LS:
        hls = 1
    print(
        "%s:%d wait=%f dmov=%d movn=%d direction=%d lls=%d hls=%d OOR=%d oldRBV=%f rbv=%f"
        % (
            tc_no,
            lineno(),
            wait,
            dmov,
            movn,
            direction,
            lls,
            hls,
            outOfRange,
            oldRBV,
            rbv,
        )
    )
    if (hls and (direction > 0)) or (lls and (direction <= 0)) or outOfRange != 0:
        print(f"{tc_no}:{int(lineno())}  STOP=1 CNEN=0 Emergency stop")
        self.axisCom.put(".STOP", 1)
        self.axisCom.put(".CNEN", 0)

    return (lls, hls, movn, dmov, outOfRange)


def waitForStart(self, tc_no, wait_for, direction, oldRBV):
    TweakValue = self.axisCom.get(".TWV")
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, tc_no + " wait waitForStart", wait_for, direction, oldRBV, TweakValue
        )
        rbv = self.axisCom.get(".RBV")
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False


def waitForStop(self, tc_no, wait_for_stop, direction, oldRBV, TweakValue):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self,
            tc_no + " wait waitForStop",
            wait_for_stop,
            direction,
            oldRBV,
            TweakValue,
        )

        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False


def tweakToLimit(self, tc_no, direction):
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
    assert direction
    old_high_limit = self.axisCom.get(".HLM", timeout=5)
    old_low_limit = self.axisCom.get(".LLM", timeout=5)
    # switch off the soft limits, save the values
    self.axisMr.setSoftLimitsOff(tc_no)

    # If we reached the limit switch, we are fine and
    # can reset the error
    self.axisMr.resetAxis(tc_no)
    print(f"{tc_no}:{int(lineno())} CNEN=1")
    self.axisMr.setCNENandWait(tc_no, 1)
    # Step through the range in 20 steps or so
    deltaToMove = (old_high_limit - old_low_limit) / 20
    maxDeltaAfterMove = deltaToMove / 2
    # soft limit range + 110 % (some test crates simulate a slit, with restrictive soft limits
    maxTweaks = (old_high_limit - old_low_limit) * 2.1 / deltaToMove
    assert maxTweaks > 0
    stopTheLoop = 0
    count = 0
    lls = 0
    hls = 0
    while not stopTheLoop:
        oldRBV = self.axisCom.get(".RBV", timeout=5)
        if direction > 0:
            destination = oldRBV + deltaToMove
        else:
            destination = oldRBV - deltaToMove
        print(
            "%s:%d Tweak motor direction=%d oldRBV=%f destination=%f"
            % (tc_no, lineno(), direction, oldRBV, destination)
        )

        self.axisCom.put(".VAL", destination)

        ret1 = waitForStart(self, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, tc_no, 10.0, direction, oldRBV, deltaToMove)
        msta = int(self.axisCom.get(".MSTA", timeout=5))
        if msta & self.axisMr.MSTA_BIT_MINUS_LS:
            stopTheLoop = 1
            lls = 1
        if msta & self.axisMr.MSTA_BIT_PLUS_LS:
            stopTheLoop = 1
            hls = 1

        rbv = self.axisCom.get(".RBV", timeout=5)
        print(
            "%s:%d STOP=1 start=%d stop=%d rbv=%f lls=%d hls=%d msta=%s"
            % (
                tc_no,
                lineno(),
                ret1,
                ret2,
                rbv,
                lls,
                hls,
                self.axisMr.getMSTAtext(msta),
            )
        )
        self.axisCom.put(".STOP", 1)
        count += 1
        if count > maxTweaks:
            stopTheLoop = 1
        if not lls and not hls:
            inPosition = self.axisMr.calcAlmostEqual(
                tc_no, destination, rbv, maxDeltaAfterMove
            )
            if not inPosition:
                stopTheLoop = 1
                print(f"{tc_no}:{int(lineno())} STOP=1 CNEN=0 Emergeny stop")
                self.axisCom.put(".STOP", 1)
                self.axisCom.put(".CNEN", 0)
        else:
            inPosition = -1

        print(
            "%s:%d old_high_limit=%f old_low_limit=%f deltaToMove=%f"
            % (tc_no, lineno(), old_high_limit, old_low_limit, deltaToMove)
        )
        print(
            "%s:%d count=%d maxTweaks=%d stopTheLoop=%d inPosition=%d"
            % (tc_no, lineno(), count, maxTweaks, stopTheLoop, inPosition)
        )

    print(
        f"{tc_no}:{int(lineno())} direction={int(direction)} hls={int(hls)} lls={int(lls)}"
    )
    # Put back the limits for the next run
    self.axisMr.setSoftLimitsOn(old_low_limit, old_high_limit)

    # Check if we reached the limit switch, prepare to move away from it
    if direction > 0:
        if not hls:
            self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
            self.assertEqual(True, hls, "hls should be active")
        newPos = rbv - deltaToMove
    else:
        if not lls:
            self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
            self.assertEqual(True, lls, "lls should be active")
        newPos = rbv + deltaToMove

    # If we reached the limit switch, we are fine and
    # can reset the error
    self.axisMr.resetAxis(tc_no)
    print(f"{tc_no}:{int(lineno())} CNEN={int(self.old_Enable)}")
    self.axisMr.setCNENandWait(tc_no, self.old_Enable)
    self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))

    # Move away from the limit switch
    self.axisCom.put(".VAL", rbv)
    self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))


class Test(unittest.TestCase):
    url_string = os.getenv("TESTEDMOTORAXIS")
    print(f"url_string={url_string}")

    axisCom = AxisCom(url_string, log_debug=True)
    axisMr = AxisMr(axisCom, url_string=url_string)

    old_Enable = int(axisCom.get(".CNEN"))
    TweakValue = axisCom.get(".TWV")
    # TWF/TWR
    def test_TC_09001(self):
        tc_no = "09001"
        print(f"{tc_no}")
        self.axisCom.put("-DbgStrToLOG", "Start " + str(tc_no))
        self.axisMr.resetAxis(tc_no)
        print(f"{tc_no}:{int(lineno())} .CNEN=1")
        self.axisMr.setCNENandWait(tc_no, 1)

        oldRBV = self.axisCom.get(".RBV")
        old_high_limit = self.axisCom.get(".HLM")
        old_low_limit = self.axisCom.get(".LLM")
        # Switch off soft limits
        self.axisMr.setSoftLimitsOff(tc_no)

        msta = int(self.axisCom.get(".MSTA"))
        direction = 0
        if msta & self.axisMr.MSTA_BIT_PLUS_LS:
            direction = -1
            destination = oldRBV - self.TweakValue
        else:
            direction = +1
            destination = oldRBV + self.TweakValue

        print(
            "%s:%d Tweak the  oldRBV=%f destination=%f"
            % (tc_no, lineno(), oldRBV, destination)
        )

        self.axisCom.put(".VAL", destination)

        ret1 = waitForStart(self, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, tc_no, 10.0, direction, oldRBV, self.TweakValue)
        msta = int(self.axisCom.get(".MSTA"))

        print(f"{tc_no} STOP=1 start={int(ret1)} stop={int(ret2)}")
        self.axisCom.put(".STOP", 1)

        self.axisCom.put(".LLM", old_low_limit)
        self.axisCom.put(".HLM", old_high_limit)
        ReadBackValue = self.axisCom.get(".RBV", use_monitor=False)
        print(f"{tc_no} destination={int(destination)} postion={ReadBackValue:f}")
        # self.assertEqual(True, ret1, 'waitForStart return True')
        self.assertEqual(True, ret2, "waitForStop return True")
        maxdelta = self.TweakValue * 2
        print(
            "%s destination=%f ReadBackValue=%f, maxdelta=%f"
            % (tc_no, destination, ReadBackValue, maxdelta)
        )
        passed = self.axisMr.calcAlmostEqual(
            tc_no, destination, ReadBackValue, maxdelta
        )
        if passed:
            self.axisCom.put("-DbgStrToLOG", "Passed " + str(tc_no))
        else:
            self.axisCom.put("-DbgStrToLOG", "Failed " + str(tc_no))
        assert passed

    def test_TC_09002(self):
        tc_no = "09002"
        print(f"{tc_no}")
        direction = 1
        tweakToLimit(self, tc_no, direction)

    def test_TC_09003(self):
        tc_no = "09003"
        print(f"{tc_no}")
        direction = -1
        tweakToLimit(self, tc_no, direction)

    def test_TC_09004(self):
        tc_no = "09004"
        print(f"{tc_no}")
        self.axisMr.resetAxis(tc_no)
        print(f"{tc_no}:{int(lineno())} CNEN={int(self.old_Enable)}")
        self.axisMr.setCNENandWait(tc_no, self.old_Enable)
        # assert False
