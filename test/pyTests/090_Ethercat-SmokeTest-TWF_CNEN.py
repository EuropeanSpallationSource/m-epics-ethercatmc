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
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
import inspect
###

polltime = 0.1

def lineno():
    return inspect.currentframe().f_back.f_lineno

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print('%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange))
    return inrange

def checkForEmergenyStop(self, motor, tc_no, wait, direction, oldRBV, TweakValue):
    outOfRange = 0
    lls = 0
    hls = 0
    outOfRange = 0
    rbv = capv_lib.capvget(motor + '.RBV', use_monitor=False)
    msta = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
    dmov = int(capv_lib.capvget(motor + '.DMOV', use_monitor=False))
    movn = int(capv_lib.capvget(motor + '.MOVN', use_monitor=False))
    if (rbv - oldRBV) > 2 * TweakValue:
        outOfRange = 1
    if (oldRBV -rbv) > 2 * TweakValue:
        outOfRange = -1
    if (msta & lib.MSTA_BIT_MINUS_LS):
        lls = 1
    if (msta & lib.MSTA_BIT_PLUS_LS):
        hls = 1

    print('%s:%d wait=%f dmov=%d movn=%d direction=%d lls=%d hls=%d OOR=%d oldRBV=%f rbv=%f' % \
        (tc_no, lineno(), wait, dmov, movn, direction, lls, hls, outOfRange, oldRBV, rbv))
    if ((hls and (direction > 0)) or
        (lls and (direction <= 0)) or
        outOfRange != 0):
        print('%s:%d  STOP=1 CNEN=0 Emergency stop' % (tc_no, lineno()))
        capv_lib.capvput(motor + '.STOP', 1)
        capv_lib.capvput(motor + '.CNEN', 0)

    return (lls, hls, movn, dmov, outOfRange)


def waitForStart(self, motor, tc_no, wait_for, direction, oldRBV):
    TweakValue = capv_lib.capvget(motor + '.TWV')
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'strt', wait_for, direction, oldRBV, TweakValue)
        rbv = capv_lib.capvget(motor + '.RBV')
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False

def resetAxis(motor, tc_no):
    wait_for_ErrRst = 5
    err = int(capv_lib.capvget(motor + '-Err', use_monitor=False))
    print('%s:%d resetAxis err=%d' % (tc_no, lineno(), err))
    #if not err:
    #    return True

    capv_lib.capvput(motor + '-ErrRst', 1)
    while wait_for_ErrRst > 0:
        wait_for_ErrRst -= polltime
        err = int(capv_lib.capvget(motor + '-Err', use_monitor=False))
        print('%s:%d wait_for_ErrRst=%f err=%d' % (tc_no, lineno(), wait_for_ErrRst, err))
        if not err:
            return True
        time.sleep(polltime)
        wait_for_ErrRst -= polltime
    return False

def waitForStop(self, motor, tc_no, wait_for_stop, direction, oldRBV, TweakValue):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'stop', wait_for_stop, direction, oldRBV, TweakValue)

        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

def tweakToLimit(self, motor, tc_no, direction):
    assert(direction)
    old_high_limit = capv_lib.capvget(motor + '.HLM', timeout=5)
    old_low_limit = capv_lib.capvget(motor + '.LLM', timeout=5)
    # switch off the soft limits, save the values
    lib.setSoftLimitsOff(motor)

    # If we reached the limit switch, we are fine and
    # can reset the error
    resetAxis(motor, tc_no)
    print('%s:%d CNEN=1' % (tc_no, lineno()))
    lib.setCNENandWait(motor, tc_no, 1)
    # Step through the range in 20 steps or so
    deltaToMove = (old_high_limit - old_low_limit) / 20
    maxDeltaAfterMove = deltaToMove / 2
    # soft limit range + 110 % (some test crates simulate a slit, with restrictive soft limits
    maxTweaks = (old_high_limit - old_low_limit) * 2.1 / deltaToMove
    assert(maxTweaks > 0)
    stopTheLoop = 0
    count = 0
    lls = 0
    hls = 0
    while not stopTheLoop:
        oldRBV = capv_lib.capvget(motor + '.RBV', timeout=5)
        if direction > 0:
            destination = oldRBV + deltaToMove
        else:
            destination = oldRBV - deltaToMove
        print('%s:%d Tweak the motor direction=%d oldRBV=%f destination=%f' \
            % (tc_no, lineno(), direction, oldRBV, destination))

        capv_lib.capvput(motor + '.VAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldRBV, deltaToMove)
        msta = int(capv_lib.capvget(motor + '.MSTA', timeout=5))
        if (msta & lib.MSTA_BIT_MINUS_LS):
            stopTheLoop = 1
            lls = 1
        if (msta & lib.MSTA_BIT_PLUS_LS):
            stopTheLoop = 1
            hls = 1

        rbv = capv_lib.capvget(motor + '.RBV', timeout=5)
        print('%s:%d STOP=1 start=%d stop=%d rbv=%f lls=%d hls=%d msta=%s' \
              % (tc_no, lineno(), ret1, ret2, rbv, lls, hls, lib.getMSTAtext(msta)))
        capv_lib.capvput(motor + '.STOP', 1)
        count += 1
        if (count > maxTweaks):
            stopTheLoop = 1
        if not lls and not hls:
            inPosition = calcAlmostEqual(motor, tc_no, destination, rbv, maxDeltaAfterMove)
            if not inPosition:
                stopTheLoop = 1
                print('%s:%d STOP=1 CNEN=0 Emergeny stop' % (tc_no, lineno()))
                capv_lib.capvput(motor + '.STOP', 1)
                capv_lib.capvput(motor + '.CNEN', 0)
        else:
            inPosition = -1

        print('%s:%d old_high_limit=%f old_low_limit=%f deltaToMove=%f' \
              % (tc_no, lineno(), old_high_limit, old_low_limit, deltaToMove))
        print('%s:%d count=%d maxTweaks=%d stopTheLoop=%d inPosition=%d' \
              % (tc_no, lineno(), count, maxTweaks, stopTheLoop, inPosition))

    print('%s:%d direction=%d hls=%d lls=%d' \
          % (tc_no, lineno(), direction, hls, lls))
    # Put back the limits for the next run
    lib.setSoftLimitsOn(motor, old_low_limit, old_high_limit)

    # Check if we reached the limit switch, prepare to move away from it
    if direction > 0:
        assert(hls)
        newPos = rbv - deltaToMove
    else:
        assert(lls)
        newPos = rbv + deltaToMove
    # If we reached the limit switch, we are fine and
    # can reset the error
    resetAxis(motor, tc_no)
    print('%s:%d CNEN=%d' % (tc_no, lineno(), self.old_Enable))
    lib.setCNENandWait(motor, tc_no, self.old_Enable)

    # Move away from the limit switch
    capv_lib.capvput(motor + '.VAL', rbv)


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")

    old_Enable = capv_lib.capvget(motor + '.CNEN')
    TweakValue = capv_lib.capvget(motor + '.TWV')

    # TWF/TWR
    def test_TC_091(self):
        tc_no = "TC-091-Tweak"
        print('%s' % tc_no)
        motor = self.motor
        resetAxis(motor, tc_no)
        print('%s:%d .CNEN=1' % (tc_no, lineno()))
        lib.setCNENandWait(motor, tc_no, 1)

        oldRBV = capv_lib.capvget(motor + '.RBV', use_monitor=False)
        old_high_limit = capv_lib.capvget(motor + '.HLM')
        old_low_limit = capv_lib.capvget(motor + '.LLM')
        # Switch off soft limits
        lib.setSoftLimitsOff(motor)

        msta = int(capv_lib.capvget(motor + '.MSTA'))
        direction = 0
        if (msta & lib.MSTA_BIT_PLUS_LS):
            direction = -1
            destination = oldRBV - self.TweakValue
        else:
            direction = +1
            destination = oldRBV + self.TweakValue

        print('%s:%d Tweak the motor oldRBV=%f destination=%f' %
              (tc_no, lineno(), oldRBV, destination))

        capv_lib.capvput(motor + '.VAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldRBV, self.TweakValue)
        msta = int(capv_lib.capvget(motor + '.MSTA'))

        print('%s STOP=1 start=%d stop=%d' % (tc_no, ret1, ret2))
        capv_lib.capvput(motor + '.STOP', 1)

        capv_lib.capvput(motor + '.LLM', old_low_limit)
        capv_lib.capvput(motor + '.HLM', old_high_limit)
        ReadBackValue = capv_lib.capvget(motor + '.RBV', use_monitor=False)
        print('%s destination=%d postion=%f' % (
            tc_no, destination, ReadBackValue))
        #self.assertEqual(True, ret1, 'waitForStart return True')
        self.assertEqual(True, ret2, 'waitForStop return True')
        maxdelta = self.TweakValue * 2
        print('%s destination=%f ReadBackValue=%f, maxdelta=%f' % (tc_no, destination, ReadBackValue, maxdelta))
        assert calcAlmostEqual(motor, tc_no, destination, ReadBackValue, maxdelta)
        #assert False

    def test_TC_092(self):
        tc_no = "TC-092-Tweak-to-HLS"
        print('%s' % tc_no)
        motor = self.motor
        direction = 1
        tweakToLimit(self, motor, tc_no, direction)

    def test_TC_093(self):
        tc_no = "TC-093-Tweak-to-LLS"
        print('%s' % tc_no)
        motor = self.motor
        direction = -1
        tweakToLimit(self, motor, tc_no, direction)

    def test_TC_094(self):
        tc_no = "TC-094-Reset-Axis"
        print('%s' % tc_no)
        motor = self.motor
        resetAxis(motor, tc_no)
        print('%s:%d CNEN=%d' % (tc_no, lineno(), self.old_Enable))
        lib.setCNENandWait(motor, tc_no, self.old_Enable)
        #assert False
