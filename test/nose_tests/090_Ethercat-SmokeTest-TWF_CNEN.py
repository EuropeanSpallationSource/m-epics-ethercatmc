#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#
# m-epics-singlemotion/src/main/test/singlemotion_test.py
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
###

epics.ca.DEFAULT_CONNECTION_TIMEOUT=10.
polltime = 0.1

def calcAlmostEqual(motor, tc_no, expected, actual, maxdelta):
    delta = math.fabs(expected - actual)
    inrange = delta < maxdelta
    print '%s: assertAlmostEqual expected=%f actual=%f delta=%f maxdelta=%f inrange=%d' % (tc_no, expected, actual, delta, maxdelta, inrange)
    return inrange

def checkForEmergenyStop(self, motor, tc_no, wait, direction, oldRBV, TweakValue):
    outOfRange = 0
    lls = 0
    hls = 0
    outOfRange = 0
    rbv = epics.caget(motor + '.RBV', use_monitor=False)
    msta = int(epics.caget(motor + '.MSTA', use_monitor=False))
    dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
    movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
    if (rbv - oldRBV) > 2 * TweakValue:
        outOfRange = 1
    if (oldRBV -rbv) > 2 * TweakValue:
        outOfRange = -1
    if (msta & self.lib.MSTA_BIT_MINUS_LS):
        lls = 1
    if (msta & self.lib.MSTA_BIT_PLUS_LS):
        hls = 1

    print '%s: wait=%f dmov=%d movn=%d lls=%d hls=%d OOR=%d oldRBV=%f rbv=%f' % \
        (tc_no, wait, dmov, movn, lls, hls, outOfRange, oldRBV, rbv)
    if ((hls and (direction > 0)) or
        (lls and (direction <= 0)) or
        outOfRange != 0):
        print '%s STOP=1 CNEN=0 Emergency stop' % (tc_no)
        epics.caput(motor + '.STOP', 1)
        epics.caput(motor + '.CNEN', 0)

    return (lls, hls, movn, dmov, outOfRange)


def waitForStart(self, motor, tc_no, wait_for, direction, oldRBV):
    TweakValue = epics.caget(motor + '.TWV')
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'strt', wait_for, direction, oldRBV, TweakValue)
        rbv = epics.caget(motor + '.RBV')
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False

def resetAxis(motor, tc_no):
    wait_for_ErrRst = 5
    err = int(epics.caget(motor + '-Err', use_monitor=False))
    print '%s: resetAxis err=%d' % (tc_no, err)
    if not err:
        return True

    epics.caput(motor + '-ErrRst', 1)
    while wait_for_ErrRst > 0:
        wait_for_ErrRst -= polltime
        err = int(epics.caget(motor + '-Err', use_monitor=False))
        print '%s: wait_for_ErrRst=%f err=%d' % (tc_no, wait_for_ErrRst, err)
        if not err:
            return True
        time.sleep(polltime)
        wait_for_ErrRst -= polltime
    assert(False)
    return False

def waitForStop(self, motor, tc_no, wait_for_stop, direction, oldRBV, TweakValue):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'stop', wait_for_stop, direction, oldRBV, TweakValue)

        if ((hls and (direction > 0)) or
            (lls and (direction <= 0)) or
            outOfRange != 0):
            return False

        if not movn and dmov:
            return True
        time.sleep(polltime)
        wait_for_stop -= polltime
    return False

def tweakToLimit(self, motor, tc_no, direction):
    assert(direction)
    oldRBV = epics.caget(motor + '.RBV', timeout=5)
    old_high_limit = epics.caget(motor + '.HLM', timeout=5)
    old_low_limit = epics.caget(motor + '.LLM', timeout=5)
    old_CNEN = epics.caget(motor + '.CNEN')
    # switch off the soft limits, save the values
    if oldRBV > 0:
        epics.caput(motor + '.LLM', 0.0)
        epics.caput(motor + '.HLM', 0.0)
    else:
        epics.caput(motor + '.HLM', 0.0)
        epics.caput(motor + '.LLM', 0.0)

    # If we reached the limit switch, we are fine and
    # can reset the error
    resetAxis(motor, tc_no)
    epics.caput(motor + '.CNEN', 1)
    # Step through the range in 20 steps or so
    deltaToMove = (old_high_limit - old_low_limit) / 20
    maxDeltaAfterMove = deltaToMove / 2
    # soft limit range + 50%
    maxTweaks = (old_high_limit - old_low_limit) * 1.5 / deltaToMove
    assert(maxTweaks > 0)
    stopTheLoop = 0
    count = 0
    lls = 0
    hls = 0
    while not stopTheLoop:
        oldRBV = epics.caget(motor + '.RBV', timeout=5)
        if direction > 0:
            destination = oldRBV + deltaToMove
        else:
            destination = oldRBV - deltaToMove
        print '%s Tweak the motor oldRBV=%f destination=%f' \
            % (tc_no, oldRBV, destination)

        epics.caput(motor + '.VAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldRBV, deltaToMove)
        msta = int(epics.caget(motor + '.MSTA', timeout=5))
        if (msta & self.lib.MSTA_BIT_MINUS_LS):
            stopTheLoop = 1
            lls = 1
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            stopTheLoop = 1
            hls = 1

        rbv = epics.caget(motor + '.RBV', timeout=5)
        print '%s STOP=1 CNEN=0 start=%d stop=%d rbv=%f lls=%d hls=%d' \
            % (tc_no, ret1, ret2, rbv, lls, hls)
        epics.caput(motor + '.STOP', 1)
        count += 1
        if (count > maxTweaks):
            stopTheLoop = 1
        if not lls and not hls:    
            inPosition = calcAlmostEqual(motor, tc_no, destination, rbv, maxDeltaAfterMove)
            if not inPosition:
                stopTheLoop = 1
                print '%s STOP=1 CNEN=0 Emergeny stop' % (tc_no)
                epics.caput(motor + '.STOP', 1)
                epics.caput(motor + '.CNEN', 0)
        else:
            inPosition = -1

        print '%s count=%d maxTweaks=%d stopTheLoop=%d inPosition=%d' \
            % (tc_no, count, maxTweaks, stopTheLoop, inPosition)
            
    print '%s direction=%d hls=%d lls=%d' \
            % (tc_no, direction, hls, lls)
    # Put back the limits for the next run
    epics.caput(motor + '.LLM', old_low_limit)
    epics.caput(motor + '.HLM', old_high_limit)

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
    print '%s CNEN=%d' % (tc_no, old_CNEN)
    epics.caput(motor + '.CNEN', old_CNEN)

    # Move away from the limit switch
    epics.caput(motor + '.JOGF', newPos, wait=True)


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    saved_HLM = epics.caget(motor + '.HLM')
    saved_LLM = epics.caget(motor + '.LLM')
    saved_CNEN = epics.caget(motor + '.CNEN')
    TweakValue = epics.caget(motor + '.TWV')

    # TWF/TWR
    def test_TC_091(self):
        tc_no = "TC-091-Tweak"
        print '%s' % tc_no
        motor = self.motor
        resetAxis(motor, tc_no)
        print '%s CNEN=1' % tc_no
        epics.caput(motor + '.CNEN', 1)

        oldRBV = epics.caget(motor + '.RBV', use_monitor=False)
        old_high_limit = epics.caget(motor + '.HLM')
        old_low_limit = epics.caget(motor + '.LLM')
        if oldRBV > 0:
            epics.caput(motor + '.LLM', 0.0)
            epics.caput(motor + '.HLM', 0.0)
        else:
            epics.caput(motor + '.HLM', 0.0)
            epics.caput(motor + '.LLM', 0.0)

        msta = int(epics.caget(motor + '.MSTA'))
        direction = 0
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            direction = -1
            destination = oldRBV - self.TweakValue
        else:
            direction = +1
            destination = oldRBV + self.TweakValue

        print '%s Tweak the motor oldRBV=%f destination=%f' % (tc_no, oldRBV, destination)

        epics.caput(motor + '.VAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldRBV, self.TweakValue)
        msta = int(epics.caget(motor + '.MSTA'))
        #print '%s STOP=1 CNEN=0 start=%d stop=%d' % (tc_no, ret1, ret2)
        #epics.caput(motor + '.CNEN', 0)
        print '%s STOP=1 start=%d stop=%d' % (tc_no, ret1, ret2)
        epics.caput(motor + '.STOP', 1)

        epics.caput(motor + '.LLM', old_low_limit)
        epics.caput(motor + '.HLM', old_high_limit)
        ReadBackValue = epics.caget(motor + '.RBV', use_monitor=False)
        print '%s destination=%d postion=%f' % (
            tc_no, destination, ReadBackValue)
        #self.assertEqual(True, ret1, 'waitForStart return True')
        self.assertEqual(True, ret2, 'waitForStop return True')
        maxdelta = self.TweakValue * 2
        print '%s destination=%f ReadBackValue=%f, maxdelta=%f' % (tc_no, destination, ReadBackValue, maxdelta)
        assert calcAlmostEqual(motor, tc_no, destination, ReadBackValue, maxdelta)
        #assert False

    def test_TC_092(self):
        tc_no = "TC-092-Tweak-to-HLS"
        print '%s' % tc_no
        motor = self.motor
        direction = 1
        tweakToLimit(self, motor, tc_no, direction)

    def test_TC_093(self):
        tc_no = "TC-093-Tweak-to-LLS"
        print '%s' % tc_no
        motor = self.motor
        direction = -1
        tweakToLimit(self, motor, tc_no, direction)

    def test_TC_094(self):
        tc_no = "TC-094-Reset-Axis"
        print '%s' % tc_no
        motor = self.motor
        resetAxis(motor, tc_no)
