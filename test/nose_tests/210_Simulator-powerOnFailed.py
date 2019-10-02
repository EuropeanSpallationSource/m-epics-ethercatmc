#!/usr/bin/env python

# EPICS Single Motion application test script
#
# http://cars9.uchicago.edu/software/python/pyepics3/
#

import epics
import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
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
    motor = os.getenv("TESTEDMOTORAXIS")
    epics.caput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])
    pv_Err   = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-Err")
    pv_nErrorId = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrId")
    pv_nErrRst = epics.PV(os.getenv("TESTEDMOTORAXIS") + "-ErrRst")
    saved_CNEN = epics.caget(motor + '.CNEN')
    saved_PwrAuto = epics.caget(motor + '-PwrAuto')

    # Jog, wait for start, power off, check error, reset error
    def test_TC_212(self):
        motor = self.motor
        tc_no = "TC-212-EnabledFailed"

        epics.caput(motor + '.CNEN', 0, wait=True)
        setValueOnSimulator(self, motor, tc_no, "bAmplifierLockedToBeOff",
                            AMPLIFIER_LOCKED_TO_BE_OFF_SILENT)
        epics.caput(motor + '-PwrAuto', 0)
        time.sleep(1.0)

        epics.caput(motor + '.CNEN', 1, wait=True)
        time.sleep(4.0)
        mstaErr = int(epics.caget(motor + '.MSTA', use_monitor=False))
        print '%s Error mstaErr=%s' % (tc_no, self.lib.getMSTAtext(mstaErr))
        self.pv_nErrRst.put(1, wait=True)

        setValueOnSimulator(self, motor, tc_no, "bAmplifierLockedToBeOff", 0)

        mstaOKagain = int(epics.caget(motor + '.MSTA', use_monitor=False))
        bError   = self.pv_Err.get(use_monitor=False)
        nErrorId = self.pv_nErrorId.get(use_monitor=False)
        print '%s Clean self.lib.MSTA_BIT_PROBLEM=%x mstaOKagain=%s bError=%d nErrorId=%d' % (tc_no, self.lib.MSTA_BIT_PROBLEM, self.lib.getMSTAtext(mstaOKagain), bError, nErrorId)

        epics.caput(motor + '.CNEN', self.saved_CNEN)
        epics.caput(motor + '-PwrAuto', self.saved_PwrAuto)

        self.assertNotEqual(0, mstaErr & self.lib.MSTA_BIT_PROBLEM, 'Error MSTA.Problem should be set)')
        self.assertEqual(0, mstaErr & self.lib.MSTA_BIT_SLIP_STALL, 'Error MSTA.Slip stall Error should not be set)')
        self.assertEqual(0, mstaErr & self.lib.MSTA_BIT_MOVING,     'Error MSTA.Moving)')

        self.assertEqual(0, mstaOKagain & self.lib.MSTA_BIT_MOVING,  'Clean MSTA.Moving)')
        self.assertEqual(0, bError,   'bError')
        self.assertEqual(0, nErrorId, 'nErrorId')


