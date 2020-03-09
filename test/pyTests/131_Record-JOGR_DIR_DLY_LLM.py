#!/usr/bin/env python
#

import unittest
import os
import sys
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    saved_DLY  = capv_lib.capvget(motor + '.DLY')
    hlm  = capv_lib.capvget(motor + '.HLM')
    llm  = capv_lib.capvget(motor + '.LLM')
    jvel = capv_lib.capvget(motor + '.JVEL')

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos    = llm + jvel + margin

    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f jog_start_pos=%f' % (llm, hlm, jog_start_pos))

    # Assert that motor is homed
    def test_TC_1311(self):
        motor = self.motor
        tc_no = "TC-1311"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')


    # per10 UserPosition
    def test_TC_1312(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1312-10-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                   tc_no, UserPosition, self.jog_start_pos, done))
            self.assertEqual(1, done,                'moveWait should return done')

    # Low soft limit JOGR
    def test_TC_1313(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1313-low-soft-limit JOGR"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 1.0)
            done = lib.jogDirection(motor, tc_no, 0)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            resW = lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            self.assertEqual(1, done,                         'DLY JOGF should be done after jogDirection')
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'DLY JOGF should not give MSTA.Problem')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY JOGF should not reach LLS')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY JOGF should not reach HLS')
            self.assertEqual(0, miss,                         'DLY JOGF should not have MISS set')
            self.assertEqual(1, resW,                         'DLY JOGF should have MIP = 0')
            self.assertEqual(1, lvio,                         'DLY JOGF should have LVIO set')

    # per10 UserPosition
    def test_TC_1314(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1314-10-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                tc_no, UserPosition, self.jog_start_pos, done))
            self.assertEqual(1, done,                         'moveWait should return done')

    def test_TC_1315(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1315-low-soft-limit JOGR"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 0.0)
            done = lib.jogDirection(motor, tc_no, 0)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            capv_lib.capvput(motor + '.JOGF', 0)
            resW = lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            self.assertEqual(True, done,                      'ndly JOGF should be done after jogDirection')
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'ndly JOGF should not give MSTA.Problem')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'ndly JOGF should not reach LLS')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'ndly JOGF should not reach HLS')
            self.assertEqual(0, miss,                         'ndly JOGF should not have MISS set')
            self.assertEqual(1, resW,                         'ndly JOGF should have MIP = 0')
            self.assertEqual(1, lvio,                         'ndly JOGF should have LVIO set')

    # Low soft limit JOGR
    def test_TC_1316(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1313-low-soft-limit JOGR"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 0.0)
            mip1  = int(capv_lib.capvget(motor + '.MIP'))
            done = lib.jogDirection(motor, tc_no, 0)

            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))
            resW = lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            mip2 = int(capv_lib.capvget(motor + '.MIP'))
            jogr = int(capv_lib.capvget(motor + '.JOGR'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            print('%s mip1=%x mip2=%x' % (
                tc_no, mip1, mip2))

            self.assertEqual(True, done, 'done should be True after jogDirection')
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'ndly2 No MSTA.Problem JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'ndly2 Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'ndly2 Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                         'ndly2 MISS not set JOGF')
            self.assertEqual(0, mip1,                         'ndly2 MIP1 not set JOGF')
            self.assertEqual(0, mip2 & lib.MIP_BIT_JOGF,      'ndly2 MIP2.JOGF not set JOGF')
            #self.assertEqual(1, resW,                        'ndly1 JOGF not set')
            self.assertEqual(0, jogr,                         'ndly2 MIP1 not set JOGR')
            self.assertEqual(1, lvio,                         'ndly2 should have LVIO set')


    # per10 UserPosition
    def test_TC_1317(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1314-10-percent-UserPosition"
            print('%s' % tc_no)
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f jog_start_pos=%f done=%s' % (
                   tc_no, UserPosition, self.jog_start_pos, done))
            self.assertEqual(1, done,                    'moveWait should return done')

    # Low soft limit JOGF + DIR
    def test_TC_1318(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1315-low-soft-limit JOGF DIR"
            print('%s' % tc_no)
            saved_DIR = capv_lib.capvget(motor + '.DIR')
            saved_FOFF = capv_lib.capvget(motor + '.FOFF')
            capv_lib.capvput(motor + '.FOFF', 1)
            capv_lib.capvput(motor + '.DIR', 1)
            done = lib.jogDirection(motor, tc_no, 1)

            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))

            capv_lib.capvput(motor + '.DIR', saved_DIR)
            capv_lib.capvput(motor + '.FOFF', saved_FOFF)

            self.assertEqual(True, done, 'done should be True after jogDirection')
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGR DIR')
