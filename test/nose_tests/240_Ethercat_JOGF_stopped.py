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

def setValueOnAxis(self, motor, tc_no, var, value):
    # Note: The driver will change Sim.this.XXX into
    # Sim.M1.XXX
    var = str(var)
    value = str(value)
    outStr = 'Main.this.' + var + '=' + value
    print '%s: DbgStrToMCU motor=%s var=%s value=%s outStr=%s' % \
          (tc_no, motor, var, value, outStr)
    assert(len(outStr) < 40)
    epics.caput(motor + '-DbgStrToMCU', outStr)


class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")
    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')
    saved_DLY  = epics.caget(motor + '.DLY')
    msta             = int(epics.caget(motor + '.MSTA'))

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    # Assert that motor is homed
    def test_TC_241(self):
        motor = self.motor
        tc_no = "TC-241"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')

    # Jog, wait for start, stop behind MR
    def test_TC_242(self):
        motor = self.motor
        tc_no = "TC-242-JOGF_stopped"
        print '%s' % tc_no

        if (self.msta & self.lib.MSTA_BIT_HOMED):
            epics.caput(motor + '.DLY', 0)
            destination = self.per10_UserPosition
            res1 = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)

            epics.caput(motor + '.JOGF', 1)
            ret2 = self.lib.waitForStart(motor, tc_no, 2.0)

            time.sleep(3)
            setValueOnAxis(self, motor, tc_no, "bExecute", 0)
            ret3 = self.lib.waitForStop(motor, tc_no, 2.0)

            val = epics.caget(motor + '.VAL')
            res4 = self.lib.verifyPosition(motor, val)
            epics.caput(motor + '.DLY', self.saved_DLY)

            self.assertNotEqual(res1 == self.__g.SUCCESS, 'move returned SUCCESS')
            #self.assertNotEqual(res2 == self.__g.SUCCESS, 'move returned SUCCESS')
            self.assertNotEqual(res4 == self.__g.SUCCESS, 'VAL synched with RBV')
