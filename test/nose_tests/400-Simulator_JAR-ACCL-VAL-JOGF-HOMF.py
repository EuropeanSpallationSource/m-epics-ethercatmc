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

polltime = 0.2

def getAcceleration(tc_no):
    print '%s: getAcceleration' % (tc_no)
    # TODO: MC_CPU1 needs to be a parameter
    epics.caput(os.getenv("TESTEDMCUASYN") + ".PORT", "MC_CPU1")

    epics.caput(os.getenv("TESTEDMCUASYN") + ".AOUT", "Main.M" + os.getenv("TESTEDMOTORADDR") + ".fAcceleration?")
    res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
    print '%s: getAcceleration res=(%s)' % (tc_no, res)
    if res == "":
        time.sleep(polltime)
        res = epics.caget (os.getenv("TESTEDMCUASYN") + ".AINP", as_string=True)
        print '%s: getAcceleration res=(%s)' % (tc_no, res)
    return float(res + "0")

class Test(unittest.TestCase):
    lib = motor_lib()
    m1 = os.getenv("TESTEDMOTORAXIS")

    saved_HLM = epics.caget(m1 + '.HLM')
    saved_LLM = epics.caget(m1 + '.LLM')
    saved_CNEN = epics.caget(m1 + '.CNEN')

    # 10% UserPosition
    def test_TC_401(self):
        motor = self.m1
        tc_no = "TC-401-10-percent-dialPosition"
        print '%s' % tc_no
        print '%s' % tc_no
        epics.caput(motor + '.CNEN', 1)
        destination =  (1 * self.saved_HLM + 9 * self.saved_LLM) / 10
        epics.caput(motor + '.VAL', destination, wait=True)

    # 20% UserPosition, check acceleration
    def test_TC_402(self):
        motor = self.m1
        tc_no = "TC-402-20-percent-dialPosition"
        print '%s' % tc_no
        saved_ACCL = epics.caget(motor + '.ACCL')

        used_ACCL = saved_ACCL + 1.0 # Make sure we have an acceleration != 0
        epics.caput(motor + '.ACCL', used_ACCL)

        destination =  (2 * self.saved_HLM + 8 * self.saved_LLM) / 10
        epics.caput(motor + '.VAL', destination, wait=True)
        rbv = epics.caget(motor + '.RBV')
        epics.caput(motor + '.ACCL', saved_ACCL)
        saved_ACCL = None

        saved_VELO = epics.caget(motor + '.VELO')
        expacc = saved_VELO / used_ACCL
        resacc = float(getAcceleration(tc_no))
        print '%s %s ACCL=%f VELO=%f expacc=%f resacc=%f' % (tc_no,motor, used_ACCL,saved_VELO,expacc,resacc)
        assert self.lib.calcAlmostEqual(motor, tc_no, expacc, resacc, 2)
        assert self.lib.calcAlmostEqual(motor, tc_no, destination, rbv, 2)


    # Jog, wait for start, stop. check fAcceleration
    def test_TC_405(self):
        motor = self.m1
        tc_no = "TC-405-JOG-fAcceleration"
        print '%s' % tc_no
        saved_JAR = epics.caget(motor + '.JAR')
        used_JAR = saved_JAR + 0.5 # Make sure we have an acceleration != 0
        epics.caput(motor + '.JAR', used_JAR)

        epics.caput(motor + '.JOGF', 1)
        ret = self.lib.waitForStart(motor, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStart return True')

        epics.caput(motor + '.JOGF', 0)
        ret = self.lib.waitForStop(motor, tc_no, 2.0)
        self.assertEqual(True, ret, 'waitForStop return True')
        epics.caput(motor + '.JAR', saved_JAR)
        saved_JAR = None

        expacc = used_JAR
        resacc = float(getAcceleration(tc_no))
        print '%s expacc=%f resacc=%f' % (tc_no,expacc,resacc)
        assert self.lib.calcAlmostEqual(motor, tc_no, expacc, resacc, 2)

