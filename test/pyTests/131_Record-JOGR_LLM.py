#!/usr/bin/env python
#

import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)

    range_postion    = hlm - llm
    jogging_velocity = capv_lib.capvget(motor + '.JVEL')
    moving_velocity  = capv_lib.capvget(motor + '.VELO')
    acceleration     = capv_lib.capvget(motor + '.ACCL')
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f per10_UserPosition=%f' % (llm, hlm, per10_UserPosition))

    # Assert if motor is not homed
    def test_TC_1311(self):
        tc_no = "TC-1311"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')


    # per90 UserPosition
    def test_TC_1312(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1312-10-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per10_UserPosition
            lib.movePosition(motor, tc_no, destination,
                                   self.moving_velocity, self.acceleration)

            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per10_UserPosition))
            assert lib.calcAlmostEqual(motor, tc_no, destination, UserPosition, 2)

    # Low soft limit JOGR
    def test_TC_1313(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1313-low-soft-limit JOGR"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.JOGR', 1, wait=True)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))

            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'No MSTA.Problem JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            capv_lib.capvput(motor + '.JOGF', 0)


    # per10 UserPosition
    def test_TC_1314(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1314-10-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per10_UserPosition
            lib.movePosition(motor, tc_no, destination,
                                  self.moving_velocity, self.acceleration)

            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per10_UserPosition))
            assert lib.calcAlmostEqual(motor, tc_no, destination, UserPosition, 2)

    # Low soft limit JOGF + DIR
    def test_TC_1315(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1315-low-soft-limit JOGF DIR"
            print('%s' % tc_no)
            saved_DIR = capv_lib.capvget(motor + '.DIR')
            saved_FOFF = capv_lib.capvget(motor + '.FOFF')
            capv_lib.capvput(motor + '.FOFF', 1)
            capv_lib.capvput(motor + '.DIR', 1)
            capv_lib.capvput(motor + '.JOGF', 1, wait=True)

            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))

            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGR DIR')
            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            capv_lib.capvput(motor + '.JOGF', 0)
            capv_lib.capvput(motor + '.DIR', saved_DIR)
            capv_lib.capvput(motor + '.FOFF', saved_FOFF)



