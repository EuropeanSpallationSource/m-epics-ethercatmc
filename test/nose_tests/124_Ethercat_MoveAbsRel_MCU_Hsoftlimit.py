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

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')

    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(epics.caget(motor + '.MSTA'))

    print 'llm=%f hlm=%f per90_UserPosition=%f' % (llm, hlm, per90_UserPosition)

    # Assert that motor is homed
    def test_TC_1241(self):
        motor = self.motor
        tc_no = "TC-1241"
        if not (self.msta & self.lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & self.lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis has been homed)')
        self.lib.initializeMotorRecordSimulatorAxis(motor, '1241')


    # per90 UserPosition
    def test_TC_1242(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1242-90-percent-UserPosition"
            print '%s' % tc_no
            destination = self.per90_UserPosition
            res = self.lib.move(motor, destination, 60)
            UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
            print '%s postion=%f per90_UserPosition=%f' % (
                tc_no, UserPosition, self.per90_UserPosition)
            self.assertEqual(res, self.__g.SUCCESS, 'move returned SUCCESS')

    # High soft limit in controller when using MoveAbs
    def test_TC_1243(self):
        motor = self.motor
        if (self.msta & self.lib.MSTA_BIT_HOMED):
            tc_no = "TC-1243-high-soft-limit Moveabs"
            print '%s' % tc_no
            drvUseEGU = epics.caget(motor + '-DrvUseEGU-RB')
            if drvUseEGU == 1:
                mres = 1.0
            else:
                mres = epics.caget(motor + '.MRES')
            rbv = epics.caget(motor + '.RBV')

            jar = epics.caget(motor + '.JAR')
            epics.caput(motor + '-ACCS', jar/mres)

            jvel = epics.caget(motor + '.JVEL')
            epics.caput(motor + '-VELO', jvel/mres)

            destination = self.hlm + 1
            timeout = self.lib.calcTimeOut(motor, destination, jvel)
            print '%s rbv=%f destination=%f timeout=%f' % (tc_no, rbv, destination, timeout)

            res = epics.caput(motor + '-MoveAbs', (destination) / mres)
            if (res == None):
                print '%s caput -Moveabs res=None' % (tc_no)
                self.assertNotEqual(res, None, 'caput -Moveabs retuned not None. PV not found ?')
            else:
                print '%s caput -Moveabs res=%d' % (tc_no, res)
                self.assertEqual(res, 1, 'caput -Moveabs returned 1')

            done = self.lib.waitForStartAndDone(motor, tc_no, timeout)

            msta = int(epics.caget(motor + '.MSTA'))
            miss = int(epics.caget(motor + '.MISS'))
            success = self.lib.verifyPosition(motor, rbv)

            if (msta & self.lib.MSTA_BIT_PROBLEM):
                epics.caput(motor + '-ErrRst', 1)

            self.assertEqual(success, self.__g.SUCCESS, 'verifyPosition returned SUCCESS')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached Moveabs')
            self.assertEqual(0, msta & self.lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached Moveabs')
            self.assertEqual(0, miss,                              'DLY MISS not set Moveabs')

