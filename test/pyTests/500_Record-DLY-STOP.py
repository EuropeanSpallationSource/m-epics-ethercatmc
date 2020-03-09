#!/usr/bin/env python
#

import unittest
import os
import sys
import time
from motor_lib import motor_lib
lib = motor_lib()
import capv_lib
###

class Test(unittest.TestCase):
    motor = os.getenv("TESTEDMOTORAXIS")
    #capv_lib.capvput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__)[0:20])

    # 10% dialPosition
    def test_TC_501(self):
        motor = str(self.motor)
        tc_no = "TC_501-10-percent-dialPosition"
        print('%s' % tc_no)
        saved_HLM = capv_lib.capvget(motor + '.HLM')
        saved_LLM = capv_lib.capvget(motor + '.LLM')

        destination =  (1 * saved_HLM + 9 * saved_LLM) / 10
        done = lib.moveWait(motor, tc_no, destination)
        self.assertEqual(1, done, 'moveWait should return done')

    # 10% dialPosition + X
    def test_TC_502(self):
        motor = self.motor
        tc_no = "TC_502-10-percent-plus-1"
        print('%s' % tc_no)
        rbv = capv_lib.capvget(motor + '.RBV')
        saved_DLY  = capv_lib.capvget(motor + '.DLY')
        saved_VELO = capv_lib.capvget(motor + '.VELO')
        saved_ACCL = capv_lib.capvget(motor + '.ACCL')

        capv_lib.capvput(motor + '.DLY',5.2)
        capv_lib.capvput(motor + '.VELO', 1)
        capv_lib.capvput(motor + '.ACCL', 1)
        capv_lib.capvput(motor + '.VAL', rbv + 1.0, wait=False)

        time.sleep(4.0)
        movn1 = capv_lib.capvget(motor + '.MOVN')
        capv_lib.capvput(motor + '.STOP', 1)
        time.sleep(7.0)
        capv_lib.capvput(motor + '.SPMG', 0)
        capv_lib.capvput(motor + '.SPMG', 3)
        time.sleep(4.0)
        dmov = capv_lib.capvget(motor + '.DMOV')
        print('%s: movn1=%d dmov=%d' % (tc_no, movn1, dmov))
        capv_lib.capvput(motor + '.DLY',  saved_DLY)
        capv_lib.capvput(motor + '.VELO', saved_VELO)
        capv_lib.capvput(motor + '.ACCL', saved_ACCL)
