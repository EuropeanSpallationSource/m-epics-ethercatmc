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


class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    saved_CNEN = capv_lib.capvget(motor + '.CNEN')
    saved_PwrAuto = capv_lib.capvget(motor + '-PwrAuto')

    # Jog, wait for start, power off, check error, reset error
    def test_TC_9402(self):
        motor = self.motor
        tc_no = "TC-9402-EnabledFailed"

        capv_lib.capvput(motor + '.CNEN', 0, wait=True)
        setValueOnSimulator(self, motor, tc_no, "bAmplifierLockedToBeOff",
                            AMPLIFIER_LOCKED_TO_BE_OFF_SILENT)
        capv_lib.capvput(motor + '-PwrAuto', 0)
        time.sleep(1.0)

        capv_lib.capvput(motor + '.CNEN', 1, wait=True)
        time.sleep(4.0)
        mstaErr = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
        print('%s Error mstaErr=%s' % (tc_no, lib.getMSTAtext(mstaErr)))
        lib.resetAxis(motor, tc_no)

        setValueOnSimulator(self, motor, tc_no, "bAmplifierLockedToBeOff", 0)

        mstaOKagain = int(capv_lib.capvget(motor + '.MSTA', use_monitor=False))
        bError   = capv_lib.capvget(motor + '-Err', use_monitor=False)
        nErrorId = capv_lib.capvget(motor + '-ErrId', use_monitor=False)
        print('%s Clean lib.MSTA_BIT_PROBLEM=%x mstaOKagain=%s bError=%d nErrorId=%d' % (tc_no, lib.MSTA_BIT_PROBLEM, lib.getMSTAtext(mstaOKagain), bError, nErrorId))

        capv_lib.capvput(motor + '.CNEN', self.saved_CNEN)
        capv_lib.capvput(motor + '-PwrAuto', self.saved_PwrAuto)

        self.assertNotEqual(0, mstaErr & lib.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, mstaErr & lib.MSTA_BIT_SLIP_STALL, 'Error MSTA.Slip stall Error should not be set)')
        self.assertEqual(0, mstaErr & lib.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        self.assertEqual(0, mstaOKagain & lib.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')


