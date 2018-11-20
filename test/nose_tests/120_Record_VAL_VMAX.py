#!/usr/bin/env python
#
# https://nose.readthedocs.org/en/latest/
# https://nose.readthedocs.org/en/latest/testing.html

import epics
import unittest
import os
import sys
from motor_lib import motor_lib
from motor_globals import motor_globals
__g = motor_globals()
###

def motorPositionTC(self, motor, tc_no, destination):
    if (self.msta & self.lib.MSTA_BIT_HOMED):
        tc_no = "TC-1202-LLM"
        print '%s' % tc_no
        res = self.lib.move(motor, destination, 60)
        UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
        print '%s postion=%f destination=%f' % (
            tc_no, UserPosition, destination)
        self.assertEqual(res, __g.SUCCESS, 'move returned SUCCESS')
        res2 = self.lib.postMoveCheck(motor)
        self.assertEqual(res2, __g.SUCCESS, 'postMoveCheck returned SUCCESS')


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")

    hlm = epics.caget(motor + '.HLM')
    llm = epics.caget(motor + '.LLM')
    velo = epics.caget(motor + '.VELO')
    vmax = epics.caget(motor + '.VMAX')
    msta             = int(epics.caget(motor + '.MSTA'))
    per10_UserPosition  = round((9 * llm + 1 * hlm) / 10)
    per90_UserPosition  = round((1 * llm + 9 * hlm) / 10)

    def test_TC_1201(self):
        motor = self.motor
        tc_no = "TC-1201"
        motorPositionTC(self, motor, tc_no, self.llm)


    def test_TC_1202(self):
        motor = self.motor
        tc_no = "TC-1202"
        motorPositionTC(self, motor, tc_no, self.per10_UserPosition)


    def test_TC_1203(self):
        motor = self.motor
        tc_no = "TC-1202"
        motorPositionTC(self, motor, tc_no, self.per90_UserPosition)

    # LLM UserPosition
    def test_TC_1204(self):
        motor = self.motor
        tc_no = "TC-1202"
        motorPositionTC(self, motor, tc_no, self.hlm)
