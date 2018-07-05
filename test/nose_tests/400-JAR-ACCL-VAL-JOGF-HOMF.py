#!/usr/bin/env python


import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
from motor_globals import motor_globals

###

polltime = 0.2

class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition  = round((8 * llm + 2 * hlm) / 10)
    msta             = int(epics.caget(motor + '.MSTA'))

    def getAcceleration(self, motor, tc_no):
        print '%s: getAcceleration %s' % (tc_no, motor)
        res = epics.caget(motor + '-Acc-RB', use_monitor=False)
        return res


    # Assert that motor is homed
    def test_TC_401(self):
        motor = self.motor
        tc_no = "TC-401"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.lib.initializeMotorRecordSimulatorAxis(motor, '1211')

    # 10% dialPosition
    def test_TC_402(self):
        tc_no = "TC-402-10-percent"
        print '%s' % tc_no
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            ret = self.lib.move(self.motor, self.per10_UserPosition, 60)
            assert (ret == 0)

    # 20% dialPosition
    def test_TC_403(self):
        tc_no = "TC-403-20-percent"
        print '%s' % tc_no
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            saved_ACCL = float(epics.caget(motor + '.ACCL'))
            saved_VELO = float(epics.caget(motor + '.VELO'))
            used_ACCL = saved_ACCL + 1.0 # Make sure we have an acceleration != 0
            epics.caput(motor + '.ACCL', used_ACCL)
            ret = self.lib.move(self.motor, self.per20_UserPosition, 60)
            resacc = self.getAcceleration(motor, tc_no)
            expacc = saved_VELO / used_ACCL
            epics.caput(motor + '.ACCL', saved_ACCL)
            print '%s ACCL=%f expacc=%f resacc=%f' % (tc_no,used_ACCL,expacc,resacc)
            assert self.lib.calcAlmostEqual(self.motor, tc_no, expacc, resacc, 2)
            assert (ret == 0)


    # JOGR
    def test_TC_404(self):
        tc_no = "TC-404-JOGR"
        print '%s' % tc_no
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            accl = float(epics.caget(motor + '.ACCL'))
            jvel = float(epics.caget(motor + '.JVEL'))
            saved_JAR = float(epics.caget(motor + '.JAR'))
            used_JAR = jvel / (accl + 2.0)
            epics.caput(motor + '.JAR', used_JAR)
            epics.caput(motor + '.JOGR', 1, wait=True)
            resacc = self.getAcceleration(motor, tc_no)
            expacc = used_JAR
            epics.caput(motor + '.JAR', saved_JAR)
            print '%s JAR=%f expacc=%f resacc=%f' % (tc_no,used_JAR,expacc,resacc)

            assert self.lib.calcAlmostEqual(self.motor, tc_no, expacc, resacc, 2)

