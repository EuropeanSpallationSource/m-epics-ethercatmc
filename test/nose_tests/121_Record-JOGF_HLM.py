#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
from motor_lib import motor_lib
###

class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    jogging_velocity = epics.caget(motor + '.JVEL')
    moving_velocity  = epics.caget(motor + '.VELO')
    acceleration     = epics.caget(motor + '.ACCL')
    msta             = int(epics.caget(motor + '.MSTA'))

    print 'llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition)

    # Assert that motor is homed
    def test_TC_1211(self):
        motor = self.motor
        tc_no = "TC-1211"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')


    # per90 UserPosition
    def test_TC_1212(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1212-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            self.lib.movePosition(motor, tc_no, destination,
                                   self.moving_velocity, self.acceleration)

            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            assert self.lib.calcAlmostEqual(motor, tc_no, destination, UserPosition, 2)

    # High soft limit JOGF
    def test_TC_1213(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1213-low-soft-limit JOGF"
            print '%s' % tc_no
            epics.caput(motor + '.JOGF', 1, wait=True)
            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))

            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'No MSTA.Problem JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            epics.caput(motor + '.JOGF', 0)


    # per90 UserPosition
    def test_TC_1214(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1214-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            self.lib.movePosition(motor, tc_no, destination,
                                  self.moving_velocity, self.acceleration)

            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            assert self.lib.calcAlmostEqual(motor, tc_no, destination, UserPosition, 2)

    # High soft limit JOGR + DIR
    def test_TC_1215(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1215-low-soft-limit JOGF DIR"
            print '%s' % tc_no
            saved_DIR = epics.caget(motor + '.DIR')
            saved_FOFF = epics.caget(motor + '.FOFF')
            epics.caput(motor + '.FOFF', 1)
            epics.caput(motor + '.DIR', 1)
            epics.caput(motor + '.JOGR', 1, wait=True)

            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))

            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF DIR')
            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            epics.caput(motor + '.JOGF', 0)
            epics.caput(motor + '.DIR', saved_DIR)
            epics.caput(motor + '.FOFF', saved_FOFF)



