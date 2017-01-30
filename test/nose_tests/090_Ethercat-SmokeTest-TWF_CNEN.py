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

def checkForEmergenyStop(self, motor, tc_no, wait, direction, oldDRBV, TweakDvalue):
    outOfRange = 0
    lls = 0
    hls = 0
    outOfRange = 0
    drbv = epics.caget(motor + '.DRBV', use_monitor=False)
    msta = int(epics.caget(motor + '.MSTA', use_monitor=False))
    dmov = int(epics.caget(motor + '.DMOV', use_monitor=False))
    movn = int(epics.caget(motor + '.MOVN', use_monitor=False))
    if (drbv - oldDRBV) > 2 * TweakDvalue:
        outOfRange = 1
    if (oldDRBV -drbv) > 2 * TweakDvalue:
        outOfRange = -1
    if (msta & self.lib.MSTA_BIT_MINUS_LS):
        lls = 1
    if (msta & self.lib.MSTA_BIT_PLUS_LS):
        hls = 1

    print '%s: wait=%f dmov=%d movn=%d lls=%d hls=%d OOR=%d oldDRBV=%f drbv=%f' % \
        (tc_no, wait, dmov, movn, lls, hls, outOfRange, oldDRBV, drbv)
    if ((hls and (direction > 0)) or
        (lls and (direction <= 0)) or
        outOfRange != 0):
        print '%s STOP=1 CNEN=0 Emergency stop' % (tc_no)
        epics.caput(motor + '.STOP', 1)
        epics.caput(motor + '.CNEN', 0)

    return (lls, hls, movn, dmov, outOfRange)


def waitForStart(self, motor, tc_no, wait_for, direction, oldDRBV):
    TweakDvalue = epics.caget(motor + '.TWV')
    while wait_for > 0:
        wait_for -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'strt', wait_for, direction, oldDRBV, TweakDvalue)
        drbv = epics.caget(motor + '.DRBV')
        if movn and not dmov:
            return True
        time.sleep(polltime)
        wait_for -= polltime
    return False

def waitForErrRst(motor, tc_no, wait_for_ErrRst):
    while wait_for_ErrRst > 0:
        wait_for_ErrRst -= polltime
        err = int(epics.caget(motor + '-Err', use_monitor=False))
        print '%s: wait_for_ErrRst=%f err=%d' % (tc_no, wait_for_ErrRst, err)
        if not err:
            return True
        time.sleep(polltime)
        wait_for_ErrRst -= polltime
    return False

def waitForStop(self, motor, tc_no, wait_for_stop, direction, oldDRBV, TweakDvalue):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        (lls, hls, movn, dmov, outOfRange) = checkForEmergenyStop(
            self, motor, tc_no + 'stop', wait_for_stop, direction, oldDRBV, TweakDvalue)

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
    oldmsta = int(epics.caget(motor + '.MSTA',  use_monitor=False))
    if (oldmsta & self.lib.MSTA_BIT_MINUS_LS):
        oldLLS = 1
    else:
        oldLLS = 0
    if (oldmsta & self.lib.MSTA_BIT_PLUS_LS):
        oldHLS = 1
    else:
        oldHLS = 0

    print '%s tweakToLimit direction=%f Amplifier=%x oldLLS=%d oldHLS=%d' % \
      (tc_no, direction, oldmsta & self.lib.MSTA_BIT_AMPON, oldLLS, oldHLS)

    assert(direction)
    oldDRBV = epics.caget(motor + '.DRBV', timeout=5)
    old_high_limit = epics.caget(motor + '.HLM', timeout=5)
    old_low_limit = epics.caget(motor + '.LLM', timeout=5)
    # switch off the soft limits, save the dvalues
    if oldDRBV > 0:
        epics.caput(motor + '.LLM', 0.0)
        epics.caput(motor + '.HLM', 0.0)
    else:
        epics.caput(motor + '.HLM', 0.0)
        epics.caput(motor + '.LLM', 0.0)

    epics.caput(motor + '.CNEN', 1)
    # Step through the range in 20 steps or so
    deltaToMove = (old_high_limit - old_low_limit) / 20
    assert(deltaToMove)
    maxDeltaAfterMove = deltaToMove / 2
    # soft limit range + 50%
    maxTweaks = (old_high_limit - old_low_limit) * 1.5 / deltaToMove
    assert(maxTweaks > 0)
    stopTheLoop = 0
    count = 0
    lls = 0
    hls = 0
    while not stopTheLoop:
        oldDRBV = epics.caget(motor + '.DRBV', timeout=5)
        if direction > 0:
            destination = oldDRBV + deltaToMove
        else:
            destination = oldDRBV - deltaToMove
        print '%s Tweak the motor oldDRBV=%f destination=%f deltaToMove=%f' \
            % (tc_no, oldDRBV, destination, deltaToMove)

        epics.caput(motor + '.DVAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldDRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldDRBV, deltaToMove)
        msta = int(epics.caget(motor + '.MSTA', timeout=5))
        if (msta & self.lib.MSTA_BIT_MINUS_LS):
            lls = 1
            if oldLLS and direction > 0:
                print '%s oldLLS=%d lls=%d' % (tc_no, oldLLS, lls)
            else:
                stopTheLoop = 1
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            hls = 1
            if oldHLS and direction <= 0:
                print '%s oldHLS=%d hls=%d' % (tc_no, oldHLS, hls)
            else:
                stopTheLoop = 1

        drbv = epics.caget(motor + '.DRBV', timeout=5)
        print '%s STOP=1 CNEN=0 start=%d stop=%d drbv=%f lls=%d hls=%d' \
            % (tc_no, ret1, ret2, drbv, lls, hls)
        epics.caput(motor + '.STOP', 1)
        count += 1
        if (count > maxTweaks):
            stopTheLoop = 1

        inPosition = calcAlmostEqual(motor, tc_no, destination, drbv, maxDeltaAfterMove)
        if not inPosition:
            stopTheLoop = 1
            print '%s STOP=1 CNEN=0 Emergeny stop' % (tc_no)
            epics.caput(motor + '.STOP', 1)
            epics.caput(motor + '.CNEN', 0)

        print '%s count=%d maxTweaks=%d stopTheLoop=%d inPosition=%d' \
            % (tc_no, count, maxTweaks, stopTheLoop, inPosition)

    # Put back the limits
    epics.caput(motor + '.LLM', old_low_limit)
    epics.caput(motor + '.HLM', old_high_limit)

    if direction > 0:
        assert(hls)
    else:
        assert(lls)


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    saved_HLM = epics.caget(motor + '.HLM')
    saved_LLM = epics.caget(motor + '.LLM')
    saved_CNEN = epics.caget(motor + '.CNEN')
    TweakDvalue = epics.caget(motor + '.TWV')

    # VAL matches RBV, especially after IOC reboot
    def test_TC_090(self):
        tc_no = "TC-090-RBV-matches-VAL"
        print '%s' % tc_no
        motor = self.motor
        rbv = epics.caget(motor + '.RBV', use_monitor=False)
        val = epics.caget(motor + '.VAL', use_monitor=False)
        maxdelta = self.TweakDvalue
        print '%s RBV=%f VAL=%f, maxdelta=%f' % (tc_no, rbv, val, maxdelta)
        assert calcAlmostEqual(motor, tc_no, rbv, val, maxdelta)

    # TWF/TWR
    def test_TC_092(self):
        tc_no = "TC-092-Tweak"
        print '%s' % tc_no
        motor = self.motor
        print '%s -ErrRst' % tc_no
        epics.caput(motor + '-ErrRst', 1)
        waitForErrRst(motor, tc_no, 1.5)
        print '%s CNEN=1' % tc_no
        epics.caput(motor + '.CNEN', 1)

        oldDRBV = epics.caget(motor + '.DRBV', use_monitor=False)
        old_high_limit = epics.caget(motor + '.HLM')
        old_low_limit = epics.caget(motor + '.LLM')
        if oldDRBV > 0:
            epics.caput(motor + '.LLM', 0.0)
            epics.caput(motor + '.HLM', 0.0)
        else:
            epics.caput(motor + '.HLM', 0.0)
            epics.caput(motor + '.LLM', 0.0)

        msta = int(epics.caget(motor + '.MSTA'))
        direction = 0
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            direction = -1
            destination = oldDRBV - self.TweakDvalue
        else:
            direction = +1
            destination = oldDRBV + self.TweakDvalue

        print '%s Tweak the motor oldDRBV=%f destination=%f' % (tc_no, oldDRBV, destination)

        epics.caput(motor + '.DVAL', destination)

        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldDRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldDRBV, self.TweakDvalue)
        msta = int(epics.caget(motor + '.MSTA'))
        print '%s STOP=1 CNEN=0 start=%d stop=%d' % (tc_no, ret1, ret2)
        epics.caput(motor + '.STOP', 1)
        epics.caput(motor + '.CNEN', 0)

        epics.caput(motor + '.LLM', old_low_limit)
        epics.caput(motor + '.HLM', old_high_limit)
        ReadBackDVAL = epics.caget(motor + '.DRBV', use_monitor=False)
        print '%s destination=%d postion=%f' % (
            tc_no, destination, ReadBackDVAL)
        #self.assertEqual(True, ret1, 'waitForStart return True')
        self.assertEqual(True, ret2, 'waitForStop return True')
        maxdelta = self.TweakDvalue * 2
        print '%s destination=%f ReadBackDVAL=%f, maxdelta=%f' % (tc_no, destination, ReadBackDVAL, maxdelta)
        assert calcAlmostEqual(motor, tc_no, destination, ReadBackDVAL, maxdelta)
        #assert False

    def test_TC_093(self):
        tc_no = "TC-093-Tweak-to-HLS"
        print '%s' % tc_no
        motor = self.motor
        direction = 1
        tweakToLimit(self, motor, tc_no, direction)

    def test_TC_094(self):
        tc_no = "TC-094-Tweak-to-LLS"
        print '%s' % tc_no
        motor = self.motor
        direction = -1
        tweakToLimit(self, motor, tc_no, direction)
