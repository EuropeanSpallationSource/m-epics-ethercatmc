#!/usr/bin/env python
#

import epics
import unittest
import os
import sys
from motor_lib import motor_lib
from motor_globals import motor_globals
###

class Test(unittest.TestCase):
    lib = motor_lib()
    __g = motor_globals()
    motor = os.getenv("TESTEDMOTORAXIS")
    epics.caput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    saved_DLY  = epics.caget(motor + '.DLY')
    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(epics.caget(motor + '.MSTA'))

    print 'llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition)

    # Assert that motor is homed
    def test_TC_1211(self):
        motor = self.motor
        tc_no = "TC-1211"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.lib.initializeMotorRecordSimulatorAxis(motor, '1211')


    # per90 UserPosition
    def test_TC_1212(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1212-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            res = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            self.assertNotEqual(res == self.__g.SUCCESS, 'move returned SUCCESS')

    # High soft limit JOGF
    def test_TC_1213(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1213-low-soft-limit JOGF"
            print '%s' % tc_no
            epics.caput(motor + '.DLY', 1.0)
            epics.caput(motor + '.JOGF', 1, wait=True)
            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))
            miss = int(epics.caget(motor + '.MISS'))

            epics.caput(motor + '.DLY', self.saved_DLY)
            epics.caput(motor + '.JOGF', 0)
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'DLY No MSTA.Problem JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'DLY MISS not set JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')

    # per90 UserPosition
    def test_TC_1214(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1214-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            res = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            self.assertNotEqual(res == self.__g.SUCCESS, 'move returned SUCCESS')

    def test_TC_1215(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1215-high-soft-limit JOGF"
            print '%s' % tc_no
            epics.caput(motor + '.DLY', 0.0)
            epics.caput(motor + '.JOGF', 1, wait=True)
            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))
            miss = int(epics.caget(motor + '.MISS'))

            epics.caput(motor + '.DLY', self.saved_DLY)
            epics.caput(motor + '.JOGF', 0)
            resW = self.lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'ndly No MSTA.Problem JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'ndly Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'ndly Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'ndly MISS not set JOGF')
            self.assertEqual(1, resW,                              'ndly resW')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')

    def test_TC_12152(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-12152-high-soft-limit JOGF"
            print '%s' % tc_no
            epics.caput(motor + '.DLY', 0.0)
            mip1  = int(epics.caget(motor + '.MIP'))
            epics.caput(motor + '.JOGF', 1, wait=True)
            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))
            miss = int(epics.caget(motor + '.MISS'))
            resW = self.lib.waitForMipZero(motor, tc_no, self.saved_DLY)
            mip2 = int(epics.caget(motor + '.MIP'))
            jogf = int(epics.caget(motor + '.JOGF'))

            epics.caput(motor + '.DLY', self.saved_DLY)
            print '%s mip1=%x mip2=%x' % (
                tc_no, mip1, mip2)

            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'ndly2 No MSTA.Problem JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'ndly2 Minus hard limit not reached JOGF')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'ndly2 Plus hard limit not reached JOGF')
            self.assertEqual(0, miss,                              'ndly2 MISS not set JOGF')
            self.assertEqual(0, mip1,                              'ndly2 MIP1 not set JOGF')
            self.assertEqual(0, mip2 & self.lib.MIP_BIT_JOGF,      'ndly2 MIP2.JOGF not set JOGF')
            #self.assertEqual(1, resW,                             'ndly1 JOGF not set')
            self.assertEqual(0, jogf,                              'ndly2 MIP1 not set JOGF')
            self.assertEqual(1, lvio, 'LVIO == 1 JOGF')


    # per90 UserPosition
    def test_TC_1216(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1216-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            res = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            self.assertNotEqual(res == self.__g.SUCCESS, 'move returned SUCCESS')

    # High soft limit JOGR + DIR
    def test_TC_1217(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1217-low-soft-limit JOGF DIR"
            print '%s' % tc_no
            saved_DIR = epics.caget(motor + '.DIR')
            saved_FOFF = epics.caget(motor + '.FOFF')
            epics.caput(motor + '.FOFF', 1)
            epics.caput(motor + '.DIR', 1)
            epics.caput(motor + '.JOGR', 1, wait=True)

            lvio = int(epics.caget(motor + '.LVIO'))
            msta = int(epics.caget(motor + '.MSTA'))

            ### commit  4efe15e76cefdc060e14dbc3 needed self.assertEqual(1, lvio, 'LVIO == 1 JOGF')
            epics.caput(motor + '.JOGF', 0)
            epics.caput(motor + '.DIR', saved_DIR)
            epics.caput(motor + '.FOFF', saved_FOFF)

            self.assertEqual(0, msta & self.lib.MSTA_BIT_PROBLEM,  'No Error MSTA.Problem JOGF DIR')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'Minus hard limit not reached JOGF DIR')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'Plus hard limit not reached JOGF DIR')
