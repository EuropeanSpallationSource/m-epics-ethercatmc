#!/usr/bin/env python


import unittest
import os
import sys
import math
import time
from motor_lib import motor_lib
lib = motor_lib()
from motor_globals import motor_globals
globals = motor_globals()
import capv_lib
###

def getAccEGUfromMCU(self, motor, tc_no):
    print('%s: getAccEGUfromMCU %s' % (tc_no, motor))
    res = capv_lib.capvget(motor + '-Acc-RB', use_monitor=False)
    return res

def check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no, vbas, velo, accl, accs, expAccEGU):
    # Put the values which the test case wanted
    if vbas > -1 :
        capv_lib.capvput(motor + '.VBAS', vbas)
    if velo > -1 :
        capv_lib.capvput(motor + '.VELO', velo)
    if accl > -1 :
        capv_lib.capvput(motor + '.ACCL', accl)
    if accs > -1 :
        capv_lib.capvput(motor + '.ACCS', accs)
    # Move the motor 2 mm (hardcoded)
    destination = 2.0 + capv_lib.capvget(motor + '.VAL')
    res = lib.move(motor, destination, 60)
    resAccEGU = getAccEGUfromMCU(self, motor, tc_no)
    print('%s: check_accEGU_ACCS_ACCL_VELO %s vbas=%f velo=%f accl=%f accs=%f expAccEGU=%f resAccEGU=%f' % \
           (tc_no, motor, vbas, velo, accl, accs, expAccEGU, resAccEGU))
    actVelo = capv_lib.capvget(motor + '.VELO', use_monitor=False)
    actAccl = capv_lib.capvget(motor + '.ACCL', use_monitor=False)
    actAccs = capv_lib.capvget(motor + '.ACCS', use_monitor=False)
    expAccs = actVelo / actAccl
    expAccl = actVelo / actAccs
    print('%s expAccl=%f expAccs=%f actVelo=%f actAccl=%f actAccs=%f' % (tc_no, expAccl, expAccs, actVelo,actAccl, actAccs))
    assert lib.calcAlmostEqual(self.motor, tc_no, expAccEGU, resAccEGU, 0.1)
    self.assertEqual(res, globals.SUCCESS, 'move returned SUCCESS')
    # Check if VELO, ACCL and ACCS are aligned
    assert lib.calcAlmostEqual(self.motor, tc_no, expAccl, actAccl, 0.1)
    assert lib.calcAlmostEqual(self.motor, tc_no, expAccs, actAccs, 0.1)

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    hlm = capv_lib.capvget(motor + '.HLM')
    llm = capv_lib.capvget(motor + '.LLM')

    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per20_UserPosition  = round((8 * llm + 2 * hlm) / 10)
    msta             = int(capv_lib.capvget(motor + '.MSTA'))
    accs             = capv_lib.capvget(motor + '.ACCS')
    homedAndPwrAndACCS = (accs != None) and (msta & lib.MSTA_BIT_HOMED) and (msta & lib.MSTA_BIT_AMPON)

    # Assert that motor is homed and has the ACCS field
    def test_TC_411(self):
        motor = self.motor
        tc_no = "TC-411"
        if not (self.homedAndPwrAndACCS):
            self.assertNotEqual(self.msta & lib.MSTA_BIT_HOMED, 0, 'Axis has been homed')
            self.assertNotEqual(self.msta & lib.MSTA_BIT_AMPON, 0, 'Amplifier is on')
            self.assertNotEqual(self.accs, None, 'ACCS field in record')

    # 10% dialPosition
    def test_TC_412(self):
        tc_no = "TC-412-10-percent"
        print('%s' % tc_no)
        motor = self.motor
        if (self.msta & lib.MSTA_BIT_HOMED):
            ret = lib.move(self.motor, self.per10_UserPosition, 60)
            assert (ret == 0)

    def test_TC_41311(self):
        tc_no = "TC-41311"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no,   0,  6.0,  0.2,   -1, 30)

    def test_TC_41312(self):
        tc_no = "TC-41312"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no,   1,  2.0,  0.2,   -1, 5)

    def test_TC_41313(self):
        tc_no = "TC-41313"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no,   1,  2.0,  0.4,   -1, 2.5)

    def test_TC_41314(self):
        tc_no = "TC-41314"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no, 4.0,  4.0,  0.5,   -1, 8.0)

    def test_TC_41315(self):
        tc_no = "TC-41315"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no, 0.0,  8.0,  0.5,   -1, 16.0)

    def test_TC_41316(self):
        tc_no = "TC-41316"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no, 0.0,  8.0, -1.0, 16.0, 16.0)

    # Keep ACCS and expAccEGU if velociy is changed
    def test_TC_41317(self):
        tc_no = "TC-41317"
        print('%s' % tc_no)
        motor = self.motor
        if (self.homedAndPwrAndACCS):
            #                                                   vbas, velo. accl, accs, expAccEGU
            check_VBAS_VELO_ACCL_ACCS_accEGU(self, motor, tc_no, 0.0,  4.0, -1.0, -1.0, 16.0)
