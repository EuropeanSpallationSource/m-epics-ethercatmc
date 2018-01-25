#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
import time
from motor_lib import motor_lib
###

class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    # 10% dialPosition
    def test_TC_501(self):
        motor = str(self.motor)
        tc_no = "TC_501-10-percent-dialPosition"
        print '%s' % tc_no
        saved_HLM = epics.caget(motor + '.HLM')
        saved_LLM = epics.caget(motor + '.LLM')

        destination =  (1 * saved_HLM + 9 * saved_LLM) / 10

        epics.caput(motor + '.VAL', destination, wait=True)
        rbv = epics.caget(motor + '.RBV')
        inpos = self.lib.calcAlmostEqual(motor, tc_no, destination, rbv, 2)
        print '%s destination=%f RBV=%f inpos=%d' % (tc_no, destination, rbv, inpos)
        assert inpos

    # 10% dialPosition + X
    def test_TC_502(self):
        motor = self.motor
        tc_no = "TC_502-10-percent-plus-1"
        print '%s' % tc_no
        rbv = epics.caget(motor + '.RBV')
        saved_DLY  = epics.caget(motor + '.DLY')
        saved_VELO = epics.caget(motor + '.VELO')
        saved_ACCL = epics.caget(motor + '.ACCL')

        epics.caput(motor + '.DLY',5.2)
        epics.caput(motor + '.VELO', 1)
        epics.caput(motor + '.ACCL', 1)
        epics.caput(motor + '.VAL', rbv + 1.0, wait=False)

        time.sleep(4.0)
        movn1 = epics.caget(motor + '.MOVN')
        epics.caput(motor + '.STOP', 1)
        time.sleep(7.0)
        epics.caput(motor + '.SPMG', 0)
        epics.caput(motor + '.SPMG', 3)
        time.sleep(4.0)
        dmov = epics.caget(motor + '.DMOV')
        print '%s: movn1=%d dmov=%d' % (tc_no, movn1, dmov)
        epics.caput(motor + '.DLY',  saved_DLY)
        epics.caput(motor + '.VELO', saved_VELO)
        epics.caput(motor + '.ACCL', saved_ACCL)
