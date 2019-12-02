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

    margin = 1.0
    # motorRecord stops jogging 1 second before reaching HLM
    jog_start_pos    = hlm - jvel - margin

    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    velo             = capv_lib.capvget(motor + '.VELO')
    accl             = capv_lib.capvget(motor + '.ACCL')

    print('llm=%f hlm=%f jog_start_pos=%f' % (llm, hlm, jog_start_pos))

    # Assert if motor is not homed
    def test_TC_1221(self):
        tc_no = "TC-1221"
        self.assertNotEqual(0, self.msta & lib.MSTA_BIT_HOMED, 'MSTA.homed (Axis is not homed)')
        self.assertNotEqual(self.llm, self.hlm, 'llm must be != hlm')



    # high limit switch
    def test_TC_1222(self):
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            tc_no = "TC-1222-high-limit-switch"
            print('%s' % tc_no)
            old_high_limit = capv_lib.capvget(motor + '.HLM')
            old_low_limit = capv_lib.capvget(motor + '.LLM')
            capv_lib.capvput(motor + '.STOP', 1)
            #Go away from limit switch
            lib.movePosition(motor, tc_no, self.jog_start_pos, self.velo, self.accl)
            destination = capv_lib.capvget(motor + '.HLM')
            rbv = capv_lib.capvget(motor + '.RBV')
            jvel = capv_lib.capvget(motor + '.JVEL')
            timeout = lib.calcTimeOut(motor, destination, jvel) * 2

            lib.setSoftLimitsOff(motor)

            done = lib.jogDirection(motor, tc_no, 1)
            # Get values, check them later
            lvio = int(capv_lib.capvget(motor + '.LVIO'))
            mstaE = int(capv_lib.capvget(motor + '.MSTA'))
            #Go away from limit switch
            lib.movePosition(motor, tc_no, old_high_limit, self.velo, self.accl)
            print('%s msta=%x lvio=%d' % (tc_no, mstaE, lvio))

            lib.setSoftLimitsOn(motor, old_low_limit, old_high_limit)

            #self.assertEqual(0, lvio, 'LVIO == 0')
            self.assertEqual(0, mstaE & lib.MSTA_BIT_PROBLEM,    'No Error MSTA.Problem at PLUS_LS')
            self.assertEqual(0, mstaE & lib.MSTA_BIT_MINUS_LS,   'Minus hard limit switch not active')
            self.assertNotEqual(0, mstaE & lib.MSTA_BIT_PLUS_LS, 'Plus hard limit switch active')


