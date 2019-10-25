#!/usr/bin/env python

# EPICS Single Motion application test script
#

import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

# Amplifier is locked to be off, but responds "OK".
# See test/simulator/EtherCAT/hw_motor.h
AMPLIFIER_LOCKED_TO_BE_OFF_SILENT = 1

def setValueOnSimulator(self, motor, tc_no, var, value):
    # Note: The driver will change Sim.this.XXX into
    # Sim.M1.XXX
    var = str(var)
    value = str(value)
    outStr = 'Sim.this.' + var + '=' + value
    print('%s: DbgStrToMCU motor=%s var=%s value=%s outStr=%s' % \
          (tc_no, motor, var, value, outStr))
    assert(len(outStr) < 40)
    capv_lib.capvput(motor + '-DbgStrToMCU', outStr)

# TODO:#
# Make sure that we have IOC:m2

class XTest(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    motor2 = "IOC:m2"
    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')
    msta= int(capv_lib.capvget(motor + '.MSTA'))

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    # Assert that motor is homed
    def Xtest_TC_231(self):
        motor = self.motor
        tc_no = "TC-231"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')

    # Jog, wait for start, power off, check error, reset error
    def Xtest_TC_232(self):
        motor = self.motor
        motor2 = self.motor2
        tc_no = "TC-232-NetYetStarted"
        print('%s' % tc_no)

        if (self.msta & lib.MSTA_BIT_HOMED):
            setValueOnSimulator(self, motor2, tc_no, "usleep", 1700000)

            destination = self.per90_UserPosition
            res1 = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))

            destination = self.per10_UserPosition
            res2 = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))

            setValueOnSimulator(self, motor2, tc_no, "usleep", 0)

            self.assertNotEqual(res1 == globals.SUCCESS, 'move returned SUCCESS')
            self.assertNotEqual(res2 == globals.SUCCESS, 'move returned SUCCESS')
