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
        print '%s STOP=1 CNEN=0 Emergeny stop' % (tc_no)
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

def waitForStop(self, motor, tc_no, wait_for_stop, direction, oldRBV):
    while wait_for_stop > 0:
        wait_for_stop -= polltime
        TweakValue = epics.caget(motor + '.TWV')
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
        print '%s -ErrRst' % tc_no
        epics.caput(motor + '-ErrRst', 1)
        waitForErrRst(motor, tc_no, 1.5)
        print '%s CNEN=1' % tc_no
        epics.caput(motor + '.CNEN', 1)

        UserPosition = epics.caget(motor + '.RBV', use_monitor=False)

        old_high_limit = epics.caget(motor + '.HLM')
        old_low_limit = epics.caget(motor + '.LLM')
        if UserPosition > 0:
            epics.caput(motor + '.LLM', 0.0)
            epics.caput(motor + '.HLM', 0.0)
        else:
            epics.caput(motor + '.HLM', 0.0)
            epics.caput(motor + '.LLM', 0.0)

	msta = int(epics.caget(motor + '.MSTA'))
        direction = 0
        if (msta & self.lib.MSTA_BIT_PLUS_LS):
            direction = -1
            destination = UserPosition - self.TweakValue
            epics.caput(motor + '.TWR', 1)
            print '%s Tweak the motor reverse' % tc_no
        else:
            direction = +1
            destination = UserPosition + self.TweakValue
            epics.caput(motor + '.TWF', 1)
            print '%s Tweak the motor forward' % tc_no

        oldRBV = epics.caget(motor + '.RBV', use_monitor=False)
        ret1 = waitForStart(self, motor, tc_no, 0.4, direction, oldRBV)
        ret2 = waitForStop(self, motor, tc_no, 10.0, direction, oldRBV)
	msta = int(epics.caget(motor + '.MSTA'))
        print '%s STOP=1 CNEN=0 start=%d stop=%d' % (tc_no, ret1, ret2)
        epics.caput(motor + '.STOP', 1)
        epics.caput(motor + '.CNEN', 0)

        epics.caput(motor + '.LLM', old_low_limit)
        epics.caput(motor + '.HLM', old_high_limit)
        UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
        print '%s destination=%d postion=%f' % (
            tc_no, destination, UserPosition)
        #self.assertEqual(True, ret1, 'waitForStart return True')
        self.assertEqual(True, ret2, 'waitForStop return True')
        maxdelta = self.TweakValue * 2
        print '%s destination=%f UserPosition=%f, maxdelta=%f' % (tc_no, destination, UserPosition, maxdelta)
        assert calcAlmostEqual(motor, tc_no, destination, UserPosition, maxdelta)
        #assert False
