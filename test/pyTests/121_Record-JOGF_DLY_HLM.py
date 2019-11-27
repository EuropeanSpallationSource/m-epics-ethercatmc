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
    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition))

    # Assert that motor is homed
    def test_TC_1211(self):
        motor = self.motor
        tc_no = "TC-1211"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        lib.initializeMotorRecordSimulatorAxis(motor, '1211')


    # per90 UserPosition
    def test_TC_1212(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1212-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per90_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))
            self.assertNotEqual(res == globals.SUCCESS, 'move returned SUCCESS')

    # High soft limit JOGF
    def test_TC_1213(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1213-low-soft-limit JOGF"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 1.0)
            capv_lib.capvput(motor + '.JOGF', 1, wait=True)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            capv_lib.capvput(motor + '.JOGF', 0)
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'DLY No MSTA.Problem JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'DLY MISS not set JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')

    # per90 UserPosition
    def test_TC_1214(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1214-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per90_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))
            self.assertNotEqual(res == globals.SUCCESS, 'move returned SUCCESS')

    def test_TC_1215(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1215-high-soft-limit JOGF"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 0.0)
            capv_lib.capvput(motor + '.JOGF', 1, wait=True)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            capv_lib.capvput(motor + '.JOGF', 0)
            resW = lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'ndly No MSTA.Problem JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'ndly Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'ndly Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'ndly MISS not set JOGF')
            self.assertEqual(1, resW,                              'ndly resW')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')

    def test_TC_12152(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-12152-high-soft-limit JOGF"
            print('%s' % tc_no)
            capv_lib.capvput(motor + '.DLY', 0.0)
            mip1  = int(capv_lib.capvget(motor + '.MIP'))
            capv_lib.capvput(motor + '.JOGF', 1, wait=True)
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))
            resW = lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            mip2 = int(capv_lib.capvget(motor + '.MIP'))
            jogf = int(capv_lib.capvget(motor + '.JOGF'))

            capv_lib.capvput(motor + '.DLY', self.saved_DLY)
            print('%s mip1=%x mip2=%x' % (
                tc_no, mip1, mip2))

            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'ndly2 No MSTA.Problem JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'ndly2 Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'ndly2 Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'ndly2 MISS not set JOGF')
            self.assertEqual(0, mip1,                              'ndly2 MIP1 not set JOGF')
            self.assertEqual(0, mip2 & lib.MIP_BIT_JOGF,      'ndly2 MIP2.JOGF not set JOGF')
            #self.assertEqual(1, resW,                             'ndly1 JOGF not set')
            self.assertEqual(0, jogf,                              'ndly2 MIP1 not set JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')


    # per90 UserPosition
    def test_TC_1216(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1216-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per90_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition))
            self.assertNotEqual(res == globals.SUCCESS, 'move returned SUCCESS')

    # High soft limit JOGR + DIR
    def test_TC_1217(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1217-low-soft-limit JOGF DIR"
            print('%s' % tc_no)
            saved_DIR = capv_lib.capvget(motor + '.DIR')
            saved_FOFF = capv_lib.capvget(motor + '.FOFF')
            capv_lib.capvput(motor + '.FOFF', 1)
            capv_lib.capvput(motor + '.DIR', 1)
            capv_lib.capvput(motor + '.JOGR', 1, wait=True)

            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            msta = int(capv_lib.capvget(motor + '.MSTA'))

            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            capv_lib.capvput(motor + '.JOGF', 0)
            capv_lib.capvput(motor + '.DIR', saved_DIR)
            capv_lib.capvput(motor + '.FOFF', saved_FOFF)

            self.assertEqual(0, msta & lib.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF DIR')
