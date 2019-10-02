#!/usr/bin/env python
#

import epics
import math
import unittest
import os
import sys
from motor_lib import motor_lib
from motor_globals import motor_globals
__g = motor_globals()
###

def motorPositionTC(self, motor, tc_no, destination, velocity):
    if velocity == 0.0:
        return
    if (self.msta & self.lib.MSTA_BIT_HOMED):
        epics.caput(motor + '-DbgStrToLOG', "Start TC " + tc_no[0:20]);
        if velocity != self.velo:
            epics.caput(motor + '.VELO', velocity)

        timeout = self.lib.calcTimeOut(motor, destination, velocity)
        res = self.lib.move(motor, destination, timeout)
        if velocity != self.velo:
            epics.caput(motor + '.VELO', self.velo)

        UserPosition = epics.caget(motor + '.RBV', use_monitor=False)
        print '%s postion=%f destination=%f' % (
            tc_no, UserPosition, destination)
        self.assertEqual(res, __g.SUCCESS, 'move returned SUCCESS')
        res2 = self.lib.postMoveCheck(motor)
        self.assertEqual(res2, __g.SUCCESS, 'postMoveCheck returned SUCCESS')


class Test(unittest.TestCase):
    lib = motor_lib()
    motor = os.getenv("TESTEDMOTORAXIS")
    epics.caput(motor + '-DbgStrToLOG', "Start " + os.path.basename(__file__))

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
        motorPositionTC(self, motor, tc_no, self.llm, self.velo)


    def test_TC_1202(self):
        motor = self.motor
        tc_no = "TC-1202"
        motorPositionTC(self, motor, tc_no, self.per10_UserPosition, self.velo)


    def test_TC_1203(self):
        motor = self.motor
        tc_no = "TC-1203"
        motorPositionTC(self, motor, tc_no, self.per90_UserPosition, self.velo)


    def test_TC_1204(self):
        motor = self.motor
        tc_no = "TC-1204"
        motorPositionTC(self, motor, tc_no, self.hlm, self.velo)

    def test_TC_1205(self):
        motor = self.motor
        tc_no = "TC-1205"
        motorPositionTC(self, motor, tc_no, self.per90_UserPosition, self.vmax)

    def test_TC_1206(self):
        motor = self.motor
        tc_no = "TC-1206"
        motorPositionTC(self, motor, tc_no, self.per10_UserPosition, self.vmax)

    def test_TC_1207(self):
        motor = self.motor
        tc_no = "TC-1207"
        motorPositionTC(self, motor, tc_no, self.llm, self.vmax)
