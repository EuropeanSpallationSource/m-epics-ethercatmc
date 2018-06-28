#!/usr/bin/env python

# EPICS Single Motion application test script
#

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
from motor_globals import motor_globals
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
    print '%s: DbgStrToMCU motor=%s var=%s value=%s outStr=%s' % \
          (tc_no, motor, var, value, outStr)
    assert(len(outStr) < 40)
    epics.caput(motor + '-DbgStrToMCU', outStr)


class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")
    motor2 = "IOC:m2"
    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')
    msta             = int(epics.caget(motor + '.MSTA'))

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    # Assert that motor is homed
    def test_TC_231(self):
        motor = self.motor
        tc_no = "TC-231"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')

    # Jog, wait for start, power off, check error, reset error
    def test_TC_232(self):
        motor = self.motor
        motor2 = self.motor2
        tc_no = "TC-232-NetYetStarted"
        print '%s' % tc_no

        if (self.msta & self.lib.MSTA_BIT_HOMED):
            setValueOnSimulator(self, motor2, tc_no, "usleep", 1700000)

            destination = self.per90_UserPosition
            res1 = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)

            destination = self.per10_UserPosition
            res2 = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
        
            setValueOnSimulator(self, motor2, tc_no, "usleep", 0)

            self.assertNotEqual(res1 == self.__g.SUCCESS, 'move returned SUCCESS')
            self.assertNotEqual(res2 == self.__g.SUCCESS, 'move returned SUCCESS')
