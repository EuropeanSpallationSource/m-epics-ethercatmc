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
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)

    range_postion    = hlm - llm
    msta             = int(capv_lib.capvget(motor + '.MSTA'))

    print('llm=%f hlm=%f per10_UserPosition=%f' % (llm, hlm, per10_UserPosition))

    # Assert that motor is homed
    def test_TC_1331(self):
        motor = self.motor
        tc_no = "TC-1331"
        if not (self.msta & lib.MSTA_BIT_HOMED):
            self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        lib.initializeMotorRecordSimulatorAxis(motor, '1331')


    # per90 UserPosition
    def test_TC_1332(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1332-90-percent-UserPosition"
            print('%s' % tc_no)
            destination = self.per10_UserPosition
            res = lib.move(motor, destination, 60)
            UserPosition = capv_lib.capvget(motor + '.RBV', use_monitor=False)
            print('%s postion=%f per10_UserPosition=%f' % (
                tc_no, UserPosition, self.per10_UserPosition))
            self.assertEqual(res, globals.SUCCESS, 'move returned SUCCESS')

    # Low soft limit in controller when using MoveVel
    def test_TC_1333(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1333-low-soft-limit MoveVel"
            print('%s' % tc_no)

            jar = capv_lib.capvget(motor + '.JAR')
            capv_lib.capvput(motor + '-ACCS', jar)

            jvel = capv_lib.capvget(motor + '.JVEL')
            res = capv_lib.capvput(motor + '-MoveVel', 0 - jvel)
            # TODO: The -MoveVel PV is not always there ?
            # Investigations needed
            #if (res == None):
            #    print('%s caput -MoveVel res=None' % (tc_no))
            #    self.assertNotEqual(res, None, 'caput -MoveVel retuned not None. PV not found ?')
            #else:
            #    print('%s caput -MoveVel res=%d' % (tc_no, res))
            #    self.assertEqual(res, 1, 'caput -MoveVel returned 1')

            time_to_wait = 180
            done = lib.waitForStartAndDone(motor, tc_no, time_to_wait)

            msta = int(capv_lib.capvget(motor + '.MSTA'))
            miss = int(capv_lib.capvget(motor + '.MISS'))

            if (msta & lib.MSTA_BIT_PROBLEM):
                capv_lib.capvput(motor + '-ErrRst', 1)

            print('%s msta=%s' % (tc_no, lib.getMSTAtext(msta)))

            echlm = capv_lib.capvget(motor + '-CfgDHLM')
            ecllm = capv_lib.capvget(motor + '-CfgDLLM')
            echlmEn = capv_lib.capvget(motor + '-CfgDHLM-En')
            ecllmEn = capv_lib.capvget(motor + '-CfgDLLM-En')
            drbv = capv_lib.capvget(motor + '.DRBV')
            print('%s echlm=%f ecllm=%f echlmEn=%d ecllmEn=%d drbv=%f'\
                % (tc_no, echlm, ecllm, echlmEn, ecllmEn, drbv))

            self.assertEqual(0, msta & lib.MSTA_BIT_MINUS_LS, 'DLY Minus hard limit not reached MoveVel')
            self.assertEqual(0, msta & lib.MSTA_BIT_PLUS_LS,  'DLY Plus hard limit not reached MoveVel')
            self.assertEqual(0, miss,                              'DLY MISS not set MoveVel')

