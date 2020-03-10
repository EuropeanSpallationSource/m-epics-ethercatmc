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
    jvel = capv_lib.capvget(motor + '.JVEL')

    margin = 1.1
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos    = llm + jvel + margin

    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    velo             = capv_lib.capvget(motor + '.VELO')
    accl             = capv_lib.capvget(motor + '.ACCL')

    print('llm=%f hlm=%f jog_start_pos=%f' % (llm, hlm, jog_start_pos))

    # Assert if motor is homed
    def test_TC_1321(self):
        motor = self.motor
        tc_no = "TC-1321"
        self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        self.assertNotEqual(self.llm, self.hlm, 'llm must be != hlm')



    # low limit switch
    def test_TC_1322(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1322-low-limit-switch"
            print('%s' % tc_no)
            old_high_limit = capv_lib.capvget(motor + '.HLM')
            old_low_limit = capv_lib.capvget(motor + '.LLM')
            rdbd  = capv_lib.capvget(motor + '.RDBD')
            capv_lib.capvput(motor + '.STOP', 1)
            #Go away from limit switch
            done = lib.moveWait(motor, tc_no, self.jog_start_pos)
            destination = capv_lib.capvget(motor + '.HLM')
            rbv = capv_lib.capvget(motor + '.RBV')
            jvel = capv_lib.capvget(motor + '.JVEL')
            timeout = lib.calcTimeOut(motor, destination, jvel) * 2

            lib.setSoftLimitsOff(motor)

            done1 = lib.jogDirection(motor, tc_no, 0)
            # Get values, check them later
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            mstaE = int(capv_lib.capvget(motor + '.MSTA'))
            #Go away from limit switch
            done2 = lib.moveWait(motor, tc_no, old_low_limit + rdbd)
            print('%s msta=%x lvio=%d' % (tc_no, mstaE, lvio))

            lib.setSoftLimitsOn(motor, old_low_limit, old_high_limit)

            self.assertEqual(0, mstaE & lib.MSTA_BIT_PROBLEM,     'MSTA.Problem should not be set')
            self.assertNotEqual(0, mstaE & lib.MSTA_BIT_MINUS_LS, 'LLS should be active')
            self.assertEqual(0, mstaE & lib.MSTA_BIT_PLUS_LS,     'HLS should not be active')
            self.assertEqual(1, done1,                            'moveWait1 should return done')
            self.assertEqual(1, done2,                            'moveWait2 should return done')


